#!/usr/bin/env python3
"""Dependency-free Modbus TCP client for the NUCLEO-F429ZI I/O panel demo.

Speaks raw Modbus TCP over a socket so every byte on the wire is easy to
follow (run with -v to print the frames).

Examples:
    mbclient.py status
    mbclient.py coil 0 on
    mbclient.py blink 250
    mbclient.py watchdog 10
    mbclient.py read hr 0 8
    mbclient.py monitor
"""

import argparse
import os
import socket
import struct
import sys
import time

FC_READ_COILS = 0x01
FC_READ_DISCRETE_INPUTS = 0x02
FC_READ_HOLDING_REGISTERS = 0x03
FC_READ_INPUT_REGISTERS = 0x04
FC_WRITE_SINGLE_COIL = 0x05
FC_WRITE_SINGLE_REGISTER = 0x06

EXCEPTIONS = {
    0x01: "illegal function",
    0x02: "illegal data address",
    0x03: "illegal data value",
    0x04: "server device failure",
}

LED_NAMES = ("green LD1", "blue LD2", "red LD3")
STATUS_FLAGS = ("WATCHDOG_TRIPPED", "TEMP_UNAVAILABLE")
TEMP_INVALID = 0x8000

HR_BLINK_PERIOD_MS = 0
HR_WATCHDOG_S = 1


class ModbusError(Exception):
    pass


class Client:
    def __init__(self, host, port=502, unit=1, timeout=3.0, verbose=False):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.unit = unit
        self.verbose = verbose
        self.tid = 0

    def close(self):
        self.sock.close()

    def _recv_exact(self, n):
        buf = b""
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("connection closed by server")
            buf += chunk
        return buf

    def request(self, fc, payload):
        """Send one PDU and return the response data (without the FC byte)."""
        self.tid = (self.tid + 1) & 0xFFFF
        pdu = bytes([fc]) + payload
        # MBAP: transaction id, protocol id (0), length (unit id + PDU), unit id
        frame = struct.pack(">HHHB", self.tid, 0, len(pdu) + 1, self.unit) + pdu
        if self.verbose:
            print(f"  -> {frame.hex(' ')}")
        self.sock.sendall(frame)

        header = self._recv_exact(7)
        tid, _proto, length, _unit = struct.unpack(">HHHB", header)
        body = self._recv_exact(length - 1)
        if self.verbose:
            print(f"  <- {(header + body).hex(' ')}")

        if tid != self.tid:
            raise ModbusError(f"transaction id mismatch (sent {self.tid}, got {tid})")
        if body[0] == fc | 0x80:
            code = body[1]
            raise ModbusError(f"exception {code:#04x} ({EXCEPTIONS.get(code, 'unknown')})")
        if body[0] != fc:
            raise ModbusError(f"unexpected function code {body[0]:#04x}")
        return body[1:]

    def read_bits(self, fc, addr, count):
        data = self.request(fc, struct.pack(">HH", addr, count))
        bits = data[1:]
        return [bool((bits[i // 8] >> (i % 8)) & 1) for i in range(count)]

    def read_registers(self, fc, addr, count):
        data = self.request(fc, struct.pack(">HH", addr, count))
        return list(struct.unpack(f">{count}H", data[1 : 1 + 2 * count]))

    def write_coil(self, addr, on):
        self.request(FC_WRITE_SINGLE_COIL, struct.pack(">HH", addr, 0xFF00 if on else 0x0000))

    def write_register(self, addr, value):
        self.request(FC_WRITE_SINGLE_REGISTER, struct.pack(">HH", addr, value))


def format_status(c):
    leds = c.read_bits(FC_READ_COILS, 0, 3)
    (button,) = c.read_bits(FC_READ_DISCRETE_INPUTS, 0, 1)
    temp, up_hi, up_lo, reqs, presses, status = c.read_registers(FC_READ_INPUT_REGISTERS, 0, 6)
    blink, watchdog = c.read_registers(FC_READ_HOLDING_REGISTERS, 0, 2)

    if temp == TEMP_INVALID:
        temp_str = "n/a"
    else:
        temp_str = f"{struct.unpack('>h', struct.pack('>H', temp))[0] / 10:.1f} °C"
    flags = [name for bit, name in enumerate(STATUS_FLAGS) if status & (1 << bit)]

    lines = [
        "LEDs:      " + ", ".join(f"{n} {'ON' if s else 'off'}" for n, s in zip(LED_NAMES, leds)),
        f"Button:    {'PRESSED' if button else 'released'} ({presses} presses)",
        f"Die temp:  {temp_str}",
        f"Uptime:    {(up_hi << 16) | up_lo} s",
        f"Requests:  {reqs}",
        f"Blink:     {f'{blink} ms' if blink else 'off (steady)'}",
        f"Watchdog:  {f'{watchdog} s' if watchdog else 'disabled'}",
        f"Status:    {', '.join(flags) if flags else 'OK'}",
    ]
    return "\n".join(lines)


READ_KINDS = {
    "coils": (FC_READ_COILS, True),
    "di": (FC_READ_DISCRETE_INPUTS, True),
    "ir": (FC_READ_INPUT_REGISTERS, False),
    "hr": (FC_READ_HOLDING_REGISTERS, False),
}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default=os.environ.get("MODBUS_HOST", "192.168.1.50"),
                        help="board IP (default: $MODBUS_HOST or 192.168.1.50)")
    parser.add_argument("--port", type=int, default=502)
    parser.add_argument("--unit", type=int, default=1)
    parser.add_argument("-v", "--verbose", action="store_true", help="print raw frames")
    sub = parser.add_subparsers(dest="cmd", required=True)

    sub.add_parser("status", help="read and print everything")

    p = sub.add_parser("monitor", help="print status repeatedly")
    p.add_argument("--interval", type=float, default=1.0)

    p = sub.add_parser("coil", help="switch an LED coil")
    p.add_argument("addr", type=int, help="0=green, 1=blue, 2=red")
    p.add_argument("state", choices=("on", "off"))

    p = sub.add_parser("blink", help="set blink period (0 = steady)")
    p.add_argument("ms", type=int)

    p = sub.add_parser("watchdog", help="set comms watchdog timeout (0 = off)")
    p.add_argument("seconds", type=int)

    p = sub.add_parser("read", help="raw read")
    p.add_argument("kind", choices=READ_KINDS)
    p.add_argument("addr", type=int)
    p.add_argument("count", type=int, nargs="?", default=1)

    p = sub.add_parser("write-hr", help="raw holding register write")
    p.add_argument("addr", type=int)
    p.add_argument("value", type=lambda s: int(s, 0))

    args = parser.parse_args()

    try:
        c = Client(args.host, args.port, args.unit, verbose=args.verbose)
    except OSError as e:
        sys.exit(f"cannot connect to {args.host}:{args.port}: {e}")

    try:
        if args.cmd == "status":
            print(format_status(c))
        elif args.cmd == "monitor":
            while True:
                print("\033[2J\033[H" + format_status(c), flush=True)
                time.sleep(args.interval)
        elif args.cmd == "coil":
            c.write_coil(args.addr, args.state == "on")
        elif args.cmd == "blink":
            c.write_register(HR_BLINK_PERIOD_MS, args.ms)
        elif args.cmd == "watchdog":
            c.write_register(HR_WATCHDOG_S, args.seconds)
        elif args.cmd == "read":
            fc, is_bits = READ_KINDS[args.kind]
            if is_bits:
                values = [int(b) for b in c.read_bits(fc, args.addr, args.count)]
            else:
                values = c.read_registers(fc, args.addr, args.count)
            for i, v in enumerate(values):
                print(f"{args.addr + i}: {v}" + ("" if is_bits else f"  (0x{v:04x})"))
        elif args.cmd == "write-hr":
            c.write_register(args.addr, args.value)
    except ModbusError as e:
        sys.exit(f"error: {e}")
    except KeyboardInterrupt:
        pass
    finally:
        c.close()


if __name__ == "__main__":
    main()
