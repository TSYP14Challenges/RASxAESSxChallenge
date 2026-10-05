#!/usr/bin/env python3
"""Build Living Map frames, and optionally inject or provision them.

  # print hex for an EVENT_BEACON (FORWARD 100cm, TURN -90deg)
  lmframe.py event --msg-id 0x1001 --seq 1 --ttl 3 --event 1 --value 200 -p F100 -p T-90

  # inject through the running TUI (it owns the bridge port)
  lmframe.py event --msg-id 0x2001 --ttl 1 --via-tui

  # inject straight into the bridge port (only when the TUI is NOT running;
  # opening the port resets the bridge, so this waits for its READY line)
  lmframe.py event --msg-id 0x2001 --ttl 1 --send /dev/cu.usbserial-BRIDGE

  # provision a beacon (raw bytes over its USB serial; beacon must be in PROVISION mode)
  lmframe.py event --msg-id 0x0A01 --ttl 3 --provision /dev/cu.usbserial-BEACON

  lmframe.py go --msg-id 0x3001 --mission 7 --send /dev/cu.usbserial-BRIDGE
  lmframe.py status --msg-id 0x4001 --payload 0102 --send ...   # reserved type 0x04
"""
import argparse
import socket
import struct
import sys
import time


def xor(data: bytes) -> int:
    x = 0
    for b in data:
        x ^= b
    return x


def primitive(spec: str) -> bytes:
    kind, mag = spec[0].upper(), int(spec[1:])
    if kind == "F":
        if not 0 <= mag <= 255:
            raise argparse.ArgumentTypeError(f"FORWARD distance out of range: {spec}")
        return bytes([0, mag])
    if kind == "T":
        if not -128 <= mag <= 127:
            raise argparse.ArgumentTypeError(f"TURN degrees out of int8 range: {spec}")
        return bytes([1, mag & 0xFF])
    raise argparse.ArgumentTypeError(f"primitive must be F<cm> or T<deg>: {spec}")


def build(args) -> bytes:
    if args.kind == "event":
        prims = b"".join(args.prim)
        body = struct.pack(">BHBBIBBB", 0x01, args.msg_id, args.seq, args.ttl, args.ts,
                           args.event, args.value, len(args.prim)) + prims
    elif args.kind == "go":
        body = struct.pack(">BHB", 0x02, args.msg_id, args.mission)
    else:
        body = struct.pack(">BH", 0x04, args.msg_id) + bytes.fromhex(args.payload)
    checksum = xor(body) ^ (0xFF if args.bad_checksum else 0)
    return body + bytes([checksum])


def open_port(path: str):
    import serial  # pip install pyserial

    s = serial.Serial()
    s.port, s.baudrate, s.timeout = path, 115200, 0.2
    s.dtr = s.rts = False
    try:
        s.open()
    except serial.SerialException as e:
        if "busy" in str(e).lower():
            sys.exit(f"{path} is busy (roidome-tui running?) - use --via-tui instead")
        raise
    return s


TUI_INJECT_ADDR = ("127.0.0.1", 47474)  # tools/roidome_tui/src/serial.rs INJECT_ADDR


def wait_for(s, prefix: str, seconds: float) -> bool:
    """Opening the port resets these boards on macOS; wait until the firmware is up."""
    end = time.time() + seconds
    while time.time() < end:
        line = s.readline().decode(errors="replace").strip()
        if line:
            print("  <", line)
        if line.startswith(prefix):
            return True
    return False


def echo(s, seconds: float):
    end = time.time() + seconds
    while time.time() < end:
        line = s.readline()
        if line:
            print("  <", line.decode(errors="replace").rstrip())


def main():
    int0 = lambda v: int(v, 0)
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="kind", required=True)

    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--msg-id", type=int0, required=True)
    common.add_argument("--bad-checksum", action="store_true", help="corrupt the checksum on purpose")
    out = common.add_mutually_exclusive_group()
    out.add_argument("--send", metavar="BRIDGE_PORT", help="inject via bridge as a TX line")
    out.add_argument("--via-tui", action="store_true", help="inject through the running roidome-tui")
    out.add_argument("--provision", metavar="BEACON_PORT", help="write raw bytes to a beacon")

    ev = sub.add_parser("event", parents=[common], help="EVENT_BEACON (0x01)")
    ev.add_argument("--seq", type=int0, default=0)
    ev.add_argument("--ttl", type=int0, default=3)
    ev.add_argument("--ts", type=int0, default=0, help="mission-relative seconds")
    ev.add_argument("--event", type=int0, default=0, help="0x00 connectivity, 0x01 gas, 0x02 fire, 0x03/0x04 victim alive/dead, 0x05 robot down")
    ev.add_argument("--value", type=int0, default=0)
    ev.add_argument("-p", "--prim", type=primitive, action="append", default=[],
                    help="F<cm> or T<deg>, repeatable")

    go = sub.add_parser("go", parents=[common], help="GO_TRIGGER (0x02)")
    go.add_argument("--mission", type=int0, default=0)

    st = sub.add_parser("status", parents=[common], help="STATUS_UPDATE (0x04, reserved)")
    st.add_argument("--payload", default="", help="opaque hex payload")

    args = ap.parse_args()
    frame = build(args)
    print(frame.hex())

    tx_line = b"TX " + frame.hex().encode() + b"\n"
    if args.via_tui:
        socket.socket(socket.AF_INET, socket.SOCK_DGRAM).sendto(tx_line, TUI_INJECT_ADDR)
        print("sent to roidome-tui (check its footer for the bridge reply)")
    elif args.send:
        with open_port(args.send) as s:
            wait_for(s, "READY", 3.0)
            s.write(tx_line)
            s.flush()
            echo(s, 1.0)
    elif args.provision:
        if args.kind != "event":
            sys.exit("only EVENT_BEACON frames can be provisioned")
        with open_port(args.provision) as s:
            if not wait_for(s, "PROVISION", 7.0):
                sys.exit("beacon never printed PROVISION (already provisioned? erase it first)")
            s.write(frame)
            s.flush()
            echo(s, 2.0)


if __name__ == "__main__":
    main()
