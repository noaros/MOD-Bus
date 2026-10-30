/*
 * Modbus TCP "I/O panel" for the NUCLEO-F429ZI.
 *
 * Exposes the board's LEDs, user button, die temperature and a few runtime
 * counters as Modbus data on TCP port 502. See README.md for the register map.
 *
 * Zephyr's Modbus stack has no TCP transport of its own, so main() runs a
 * small socket server that unwraps the MBAP header and hands each request to
 * the Modbus server through a RAW interface.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/modbus/modbus.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(io_panel, LOG_LEVEL_INF);

#define MODBUS_TCP_PORT       502
#define MODBUS_UNIT_ID        1
#define RESPONSE_TIMEOUT_MS   1000
#define CLIENT_IDLE_TIMEOUT_S 30

#define IO_TICK_MS        20
#define TEMP_PERIOD_MS    1000
#define IO_STACK_SIZE     1536
#define IO_PRIORITY       7

#define BLINK_MIN_MS      50
#define BLINK_MAX_MS      5000
#define WATCHDOG_MAX_S    3600

#define TEMP_INVALID      0x8000

/* Input register addresses */
enum {
	IR_DIE_TEMP,       /* 0.1 degC, signed; 0x8000 = unavailable */
	IR_UPTIME_HI,      /* uptime in seconds, high word */
	IR_UPTIME_LO,      /* uptime in seconds, low word */
	IR_REQUESTS,       /* requests served (wraps) */
	IR_BUTTON_PRESSES, /* user button presses (wraps) */
	IR_STATUS,         /* STATUS_BIT_* flags */
	IR_COUNT,
};

/* Holding register addresses */
enum {
	HR_BLINK_PERIOD_MS, /* 0 = steady, else blink "on" coils at this period */
	HR_WATCHDOG_S,      /* 0 = off, else clear coils after N s without requests */
	HR_SCRATCH_FIRST,   /* free-for-all scratch registers */
	HR_COUNT = HR_SCRATCH_FIRST + 6,
};

/* Bits in IR_STATUS */
enum {
	STATUS_BIT_WATCHDOG_TRIPPED, /* cleared by writing any coil */
	STATUS_BIT_TEMP_UNAVAILABLE,
};

static const struct gpio_dt_spec leds[] = {
	GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios), /* LD1 green */
	GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios), /* LD2 blue */
	GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios), /* LD3 red */
};
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);

#if DT_NODE_HAS_STATUS(DT_NODELABEL(die_temp), okay)
static const struct device *const die_temp = DEVICE_DT_GET(DT_NODELABEL(die_temp));
#else
static const struct device *const die_temp;
#endif

/*
 * Shared between the Modbus server (system work queue), the TCP loop and the
 * I/O thread. Holding registers are only written by the Modbus server, and
 * 16-bit aligned loads are atomic on Cortex-M, so they need no lock.
 */
static atomic_t coils;
static atomic_t requests;
static atomic_t button_presses;
static atomic_t status;
static atomic_t last_request_ms;
static atomic_t die_temp_dc = ATOMIC_INIT(TEMP_INVALID);
static uint16_t holding[HR_COUNT];

/* ---- Modbus server callbacks ---- */

static int coil_rd(uint16_t addr, bool *state)
{
	if (addr >= ARRAY_SIZE(leds)) {
		return -ENOTSUP;
	}

	*state = atomic_test_bit(&coils, addr);
	return 0;
}

static int coil_wr(uint16_t addr, bool state)
{
	if (addr >= ARRAY_SIZE(leds)) {
		return -ENOTSUP;
	}

	atomic_set_bit_to(&coils, addr, state);
	atomic_clear_bit(&status, STATUS_BIT_WATCHDOG_TRIPPED);
	LOG_INF("Coil %u -> %s", addr, state ? "ON" : "OFF");
	return 0;
}

static int discrete_input_rd(uint16_t addr, bool *state)
{
	int val;

	if (addr != 0) {
		return -ENOTSUP;
	}

	val = gpio_pin_get_dt(&button);
	if (val < 0) {
		return val;
	}

	*state = val;
	return 0;
}

static int input_reg_rd(uint16_t addr, uint16_t *reg)
{
	uint32_t uptime_s = k_uptime_get() / MSEC_PER_SEC;

	switch (addr) {
	case IR_DIE_TEMP:
		*reg = (uint16_t)atomic_get(&die_temp_dc);
		break;
	case IR_UPTIME_HI:
		*reg = uptime_s >> 16;
		break;
	case IR_UPTIME_LO:
		*reg = uptime_s & 0xFFFF;
		break;
	case IR_REQUESTS:
		*reg = (uint16_t)atomic_get(&requests);
		break;
	case IR_BUTTON_PRESSES:
		*reg = (uint16_t)atomic_get(&button_presses);
		break;
	case IR_STATUS:
		*reg = (uint16_t)atomic_get(&status);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int holding_reg_rd(uint16_t addr, uint16_t *reg)
{
	if (addr >= HR_COUNT) {
		return -ENOTSUP;
	}

	*reg = holding[addr];
	return 0;
}

static int holding_reg_wr(uint16_t addr, uint16_t reg)
{
	if (addr >= HR_COUNT) {
		return -ENOTSUP;
	}

	/* Out-of-range values are rejected with an exception response */
	if (addr == HR_BLINK_PERIOD_MS && reg != 0 &&
	    (reg < BLINK_MIN_MS || reg > BLINK_MAX_MS)) {
		return -EINVAL;
	}
	if (addr == HR_WATCHDOG_S && reg > WATCHDOG_MAX_S) {
		return -EINVAL;
	}

	holding[addr] = reg;
	LOG_INF("Holding register %u -> %u", addr, reg);
	return 0;
}

static struct modbus_user_callbacks mbs_cbs = {
	.coil_rd = coil_rd,
	.coil_wr = coil_wr,
	.discrete_input_rd = discrete_input_rd,
	.input_reg_rd = input_reg_rd,
	.holding_reg_rd = holding_reg_rd,
	.holding_reg_wr = holding_reg_wr,
};

/* ---- I/O thread: LEDs, button, die temperature, comms watchdog ---- */

static void update_die_temp(void)
{
	struct sensor_value val;
	int32_t dc;

	if (die_temp == NULL || !device_is_ready(die_temp) ||
	    sensor_sample_fetch(die_temp) < 0 ||
	    sensor_channel_get(die_temp, SENSOR_CHAN_DIE_TEMP, &val) < 0) {
		atomic_set(&die_temp_dc, TEMP_INVALID);
		atomic_set_bit(&status, STATUS_BIT_TEMP_UNAVAILABLE);
		return;
	}

	dc = val.val1 * 10 + val.val2 / 100000;
	atomic_set(&die_temp_dc, (uint16_t)(int16_t)dc);
	atomic_clear_bit(&status, STATUS_BIT_TEMP_UNAVAILABLE);
}

static void watchdog_check(uint32_t now)
{
	uint16_t timeout_s = holding[HR_WATCHDOG_S];

	if (timeout_s == 0 || atomic_test_bit(&status, STATUS_BIT_WATCHDOG_TRIPPED)) {
		return;
	}

	if (now - (uint32_t)atomic_get(&last_request_ms) > timeout_s * MSEC_PER_SEC) {
		atomic_clear(&coils);
		atomic_set_bit(&status, STATUS_BIT_WATCHDOG_TRIPPED);
		LOG_WRN("No Modbus requests for %u s, outputs forced off", timeout_s);
	}
}

static void io_thread(void *p1, void *p2, void *p3)
{
	bool was_pressed = false;
	uint32_t next_temp_ms = 0;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		uint32_t now = k_uptime_get_32();
		uint16_t period = holding[HR_BLINK_PERIOD_MS];
		bool blink_on = period == 0 || (now % period) < period / 2;
		int pressed;

		for (size_t i = 0; i < ARRAY_SIZE(leds); i++) {
			gpio_pin_set_dt(&leds[i], atomic_test_bit(&coils, i) && blink_on);
		}

		pressed = gpio_pin_get_dt(&button);
		if (pressed > 0 && !was_pressed) {
			atomic_inc(&button_presses);
		}
		was_pressed = pressed > 0;

		watchdog_check(now);

		if ((int32_t)(now - next_temp_ms) >= 0) {
			update_die_temp();
			next_temp_ms = now + TEMP_PERIOD_MS;
		}

		k_msleep(IO_TICK_MS);
	}
}

K_THREAD_STACK_DEFINE(io_stack, IO_STACK_SIZE);
static struct k_thread io_thread_data;

static int init_gpio(void)
{
	int err;

	for (size_t i = 0; i < ARRAY_SIZE(leds); i++) {
		if (!gpio_is_ready_dt(&leds[i])) {
			return -ENODEV;
		}
		err = gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT_INACTIVE);
		if (err < 0) {
			return err;
		}
	}

	if (!gpio_is_ready_dt(&button)) {
		return -ENODEV;
	}
	return gpio_pin_configure_dt(&button, GPIO_INPUT);
}

/* ---- Modbus TCP transport ---- */

static int server_iface;
static struct modbus_adu adu;
static uint8_t frame[MODBUS_MBAP_AND_FC_LENGTH + sizeof(adu.data)];
static K_SEM_DEFINE(response_ready, 0, 1);

static int raw_tx_cb(const int iface, const struct modbus_adu *rsp, void *user_data)
{
	ARG_UNUSED(iface);
	ARG_UNUSED(user_data);

	adu = *rsp;
	k_sem_give(&response_ready);
	return 0;
}

static int init_modbus_server(void)
{
	const struct modbus_iface_param param = {
		.mode = MODBUS_MODE_RAW,
		.server = {
			.user_cb = &mbs_cbs,
			.unit_id = MODBUS_UNIT_ID,
		},
		.rawcb = {
			.raw_tx_cb = raw_tx_cb,
			.user_data = NULL,
		},
	};

	server_iface = modbus_iface_get_by_name("RAW_0");
	if (server_iface < 0) {
		return -ENODEV;
	}

	return modbus_init_server(server_iface, param);
}

static int recv_all(int sock, void *buf, size_t len)
{
	ssize_t rc = zsock_recv(sock, buf, len, ZSOCK_MSG_WAITALL);

	if (rc == 0) {
		return -ENOTCONN;
	}
	if (rc < 0) {
		return -errno;
	}
	return rc == len ? 0 : -EIO;
}

static int send_all(int sock, const uint8_t *buf, size_t len)
{
	while (len > 0) {
		ssize_t rc = zsock_send(sock, buf, len, 0);

		if (rc < 0) {
			return -errno;
		}
		buf += rc;
		len -= rc;
	}

	return 0;
}

static int handle_request(int client)
{
	uint16_t proto_id;
	uint16_t mbap_len;
	uint8_t req_unit_id;
	int err;

	err = recv_all(client, frame, MODBUS_MBAP_AND_FC_LENGTH);
	if (err) {
		return err;
	}

	/* Validate before modbus_raw_get_header(), which would silently clamp */
	proto_id = sys_get_be16(&frame[2]);
	mbap_len = sys_get_be16(&frame[4]);
	if (proto_id != 0 || mbap_len < 2 || mbap_len - 2 > sizeof(adu.data)) {
		LOG_WRN("Malformed MBAP header (proto %u, len %u)", proto_id, mbap_len);
		return -EBADMSG;
	}

	modbus_raw_get_header(&adu, frame);
	if (adu.length > 0) {
		err = recv_all(client, adu.data, adu.length);
		if (err) {
			return err;
		}
	}

	atomic_inc(&requests);
	atomic_set(&last_request_ms, k_uptime_get_32());

	/*
	 * A Modbus TCP device addressed directly is reached whatever unit ID the
	 * client sends (commonly 0, 1 or 255). Answer as our unit, but echo the
	 * client's ID back in the response.
	 */
	req_unit_id = adu.unit_id;
	adu.unit_id = MODBUS_UNIT_ID;

	k_sem_reset(&response_ready);
	if (modbus_raw_submit_rx(server_iface, &adu) != 0) {
		LOG_ERR("Failed to submit request to Modbus server");
		return -EIO;
	}

	if (k_sem_take(&response_ready, K_MSEC(RESPONSE_TIMEOUT_MS)) != 0) {
		LOG_ERR("Modbus server did not respond");
		modbus_raw_set_server_failure(&adu);
	}

	adu.unit_id = req_unit_id;
	modbus_raw_put_header(&adu, frame);
	memcpy(&frame[MODBUS_MBAP_AND_FC_LENGTH], adu.data, adu.length);

	return send_all(client, frame, MODBUS_MBAP_AND_FC_LENGTH + adu.length);
}

static void serve_client(int client)
{
	const struct zsock_timeval idle = { .tv_sec = CLIENT_IDLE_TIMEOUT_S };
	int err;

	/* Drop silent clients so they can't hold the single connection slot */
	zsock_setsockopt(client, ZSOCK_SOL_SOCKET, ZSOCK_SO_RCVTIMEO, &idle, sizeof(idle));

	do {
		err = handle_request(client);
	} while (err == 0);

	LOG_INF("Client disconnected (%d)", err);
	zsock_close(client);
}

int main(void)
{
	struct sockaddr_in bind_addr = {
		.sin_family = AF_INET,
		.sin_port = htons(MODBUS_TCP_PORT),
		.sin_addr.s_addr = htonl(INADDR_ANY),
	};
	int server;
	int err;

	err = init_gpio();
	if (err < 0) {
		LOG_ERR("GPIO init failed (%d)", err);
		return err;
	}

	err = init_modbus_server();
	if (err < 0) {
		LOG_ERR("Modbus server init failed (%d)", err);
		return err;
	}

	k_thread_create(&io_thread_data, io_stack, K_THREAD_STACK_SIZEOF(io_stack),
			io_thread, NULL, NULL, NULL, IO_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&io_thread_data, "io");

	server = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (server < 0) {
		LOG_ERR("socket() failed (%d)", errno);
		return -errno;
	}

	if (zsock_bind(server, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0 ||
	    zsock_listen(server, 1) < 0) {
		LOG_ERR("bind/listen failed (%d)", errno);
		zsock_close(server);
		return -errno;
	}

	LOG_INF("Modbus TCP server listening on port %d (unit %d)",
		MODBUS_TCP_PORT, MODBUS_UNIT_ID);

	/* One client at a time keeps the demo simple */
	while (true) {
		struct sockaddr_in client_addr;
		socklen_t addr_len = sizeof(client_addr);
		char addr_str[NET_IPV4_ADDR_LEN];
		int client;

		client = zsock_accept(server, (struct sockaddr *)&client_addr, &addr_len);
		if (client < 0) {
			LOG_ERR("accept() failed (%d)", errno);
			continue;
		}

		zsock_inet_ntop(AF_INET, &client_addr.sin_addr, addr_str, sizeof(addr_str));
		LOG_INF("Client connected from %s", addr_str);

		serve_client(client);
	}

	return 0;
}
