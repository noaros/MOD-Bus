# Modbus

My new preferred way of writing software with Claude is in terms of many small iterations, not unlike how I would have done it without Claude. However, I accidentally created this project in one pass. I then used Claude to make up iterations for it after the fact, and used this as a basis for learning, asking further questions.

1. **Blinky over the network.** Started from a bare Zephyr app on the NUCLEO-F429ZI
   with Ethernet on a static IP. The goal was just a successful `ping 192.168.1.50`.
2. **Raw TCP echo on port 502.** Added a single-client socket loop that accepted a
   connection and echoed bytes back. This proved that the TCP stack and the
   LAN8742 PHY worked before any Modbus code was involved.
3. **MBAP framing.** Taught the loop to read the 7-byte MBAP header and the function
   code, to check the length field and to drop malformed frames. Replies were
   still hard-coded.
4. **Zephyr Modbus server through a RAW interface.** Connected the socket loop to
   Zephyr's Modbus subsystem (`RAW_0`). Requests are passed in, and the response
   callback re-adds the MBAP header. The first real function: coils 0–2 drive the
   three LEDs.
5. **Inputs.** Added the user button as discrete input 0, and input registers for
   uptime, request count and button presses.
6. **Die temperature.** Enabled the internal temperature sensor through a board
   overlay and exposed it in 0.1 °C, with `0x8000` and a status bit for when it
   is unavailable.
7. **Holding registers and the I/O thread.** Moved the LED handling into a
   low-priority thread and added the blink period register and scratch registers.
   Out-of-range writes return Modbus exceptions.
8. **Comms watchdog.** Holding register 1 sets a timeout. When no request arrives
   within it, the outputs are forced off and status bit 0 is set. Also added a
   30 s idle disconnect so that a stale client can't block the server.
9. **PC-side client.** Wrote `tools/mbclient.py`, a dependency-free client with
   `status`, `monitor`, `coil`, `blink` and `watchdog` commands, raw reads and
   writes, and `-v` frame dumps for debugging.
10. **DHCP option.** Added `dhcp.conf` as an overlay so that the board can join an
    existing LAN without editing `prj.conf`.
11. **Zephyr v4.2.0 and a minimal workspace.** Updated the socket option names
    for v4.2.0. Added `west.yml` so that a workspace can be created with only
    Zephyr and the STM32 HAL.
12. **Connection limits.** Connections that the board closed itself waited in
    TIME_WAIT, and after three of them in a row the 4 default slots were used up
    and new clients were refused. Raised `CONFIG_NET_MAX_CONN` and
    `CONFIG_NET_MAX_CONTEXTS` to leave headroom.

=====================================================================


A Modbus TCP "I/O panel" demo for the **NUCLEO-F429ZI**, built on **Zephyr RTOS**.
The board serves Modbus TCP on port 502 over its on-board Ethernet and exposes its
LEDs, user button, die temperature and some runtime counters as Modbus data.

## Register map (unit ID 1, zero-based addresses)

| Table | Addr | Meaning | Access |
|---|---|---|---|
| Coils | 0 / 1 / 2 | LD1 green / LD2 blue / LD3 red | R/W |
| Discrete inputs | 0 | User button B1 (1 = pressed) | R |
| Input registers | 0 | Die temperature, 0.1 °C, signed (`0x8000` = unavailable) | R |
| | 1, 2 | Uptime in seconds, 32-bit, high word first | R |
| | 3 | Requests served (wraps at 65535) | R |
| | 4 | Button presses (wraps) | R |
| | 5 | Status bits: bit0 watchdog tripped, bit1 temperature unavailable | R |
| Holding registers | 0 | Blink period in ms: `0` = steady, `50`–`5000` = "on" LEDs blink | R/W |
| | 1 | Comms watchdog in s: `0` = off, `1`–`3600` | R/W |
| | 2–7 | Scratch registers (any value) | R/W |

**Comms watchdog:** if holding register 1 is non-zero and no request arrives for
that many seconds, all coils are forced off and status bit 0 is set. Writing any
coil clears the flag.

Out-of-range addresses or values get a Modbus exception response.
The server answers whatever unit ID the client sends and echoes it back.

## Hardware

- NUCLEO-F429ZI and a micro-USB cable to the ST-LINK port (CN1)
- Ethernet cable to your router/switch, or directly to your PC

## Build and flash

You need a Zephyr workspace. Pick one of these options:

**Option A: you already have a Zephyr workspace** (for example `~/zephyrproject`)

```sh
cd ~/zephyrproject
west build -b nucleo_f429zi /data/Modbus -p
west flash
```

**Option B: minimal workspace from this repo's `west.yml`** (Zephyr v4.2.0 plus the STM32 HAL only)

The workspace lives in `~/modbus-ws` and links to this repo, so there is only
one copy of the code. The workspace config is written by hand because `west init -l`
follows the symlink and would create the workspace in this repo's parent directory.

```sh
mkdir ~/modbus-ws && cd ~/modbus-ws
ln -s /data/Modbus Modbus
mkdir .west && printf '[manifest]\npath = Modbus\nfile = west.yml\n\n[zephyr]\nbase = zephyr\n' > .west/config

python3 -m venv .venv && . .venv/bin/activate
pip install west ninja
west update --narrow -o=--depth=1      # about 1.6 GB
pip install -r zephyr/scripts/requirements-base.txt
west zephyr-export

west build -b nucleo_f429zi Modbus -d build -p
west flash -d build
```

For later sessions, `cd ~/modbus-ws && . .venv/bin/activate` (plus the toolchain
exports below), then `west build -d build` to rebuild.

Toolchain: install the Zephyr SDK, **or** reuse an existing Arm GNU toolchain:

```sh
export ZEPHYR_TOOLCHAIN_VARIANT=gnuarmemb
export GNUARMEMB_TOOLCHAIN_PATH=~/arm15.3
```

Since STM32CubeProgrammer is installed, `west flash -r stm32cubeprogrammer` also works.

**Serial console:** the ST-LINK virtual COM port runs at 115200 8N1, usually
`/dev/ttyACM0`. Use for example `picocom -b 115200 /dev/ttyACM0`.

## Network setup

The default is a static address of **192.168.1.50/24**, set in `prj.conf`. Change
it to match your network, or:

- **DHCP:** `west build -b nucleo_f429zi Modbus -d build -p -- -DEXTRA_CONF_FILE=dhcp.conf`. The leased
  address is logged on the serial console.
- **Direct to PC:** give the PC's wired port a static address in the same subnet,
  for example `192.168.1.10/24`.

Check that the board is reachable with `ping 192.168.1.50`.

## Try it

`tools/mbclient.py` is a dependency-free client. Add `-v` to see the raw frames on the wire.

```sh
export MODBUS_HOST=192.168.1.50
./tools/mbclient.py status          # dump everything
./tools/mbclient.py coil 0 on       # green LED on
./tools/mbclient.py blink 250       # lit LEDs blink at 4 Hz
./tools/mbclient.py watchdog 10     # outputs off after 10 s of silence
./tools/mbclient.py monitor         # live view; press the blue button
./tools/mbclient.py -v read ir 0 6  # raw read with frame hex dump
./tools/mbclient.py blink 10        # -> exception 0x02 (value rejected)
```

With `mbpoll`, note that it uses one-based references by default, so `-r 1` means address 0:

```sh
mbpoll -m tcp -a 1 -t 0 -r 1 -c 3 -1 192.168.1.50     # read coils
mbpoll -m tcp -a 1 -t 0 -r 2 192.168.1.50 1           # blue LED on
mbpoll -m tcp -a 1 -t 3 -r 1 -c 6 -1 192.168.1.50     # input registers
```

Wireshark decodes the traffic natively. Use the filter `mbtcp`.

## How it works

Zephyr's Modbus subsystem provides the server logic (function codes, exceptions)
but has no TCP transport. `src/main.c` therefore runs a small socket server that:

1. reads the 7-byte MBAP header and the function code, and validates the length,
2. passes the request to the Modbus server through a RAW interface (`RAW_0`),
3. waits for the response callback, re-adds the MBAP header and sends it back.

A separate low-priority thread drives the LEDs and blinking, counts button
presses, samples the die temperature once per second and runs the comms watchdog.

Limitations: the server handles one TCP client at a time. A client that stays
idle for 30 s is disconnected.

## Layout

```
CMakeLists.txt               Zephyr app definition
prj.conf                     Kconfig: networking, Modbus, peripherals
dhcp.conf                    optional DHCP overlay config
boards/nucleo_f429zi.overlay enables the die temperature sensor
src/main.c                   firmware
tools/mbclient.py            PC-side test client
west.yml                     optional minimal west manifest
```
