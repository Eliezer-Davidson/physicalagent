#!/usr/bin/env python3
"""Serial-to-TCP bridge for the AiHand. Runs ON THE ROBOT'S COMPUTER.

The hand's micro:bit V2 is plugged into the TonyPi's Raspberry Pi, so its serial
device only exists there. This forwards a TCP port on the Pi to that device, and
TcpAiHandLink on the operator's laptop dials it.

    # on the Pi
    python3 aihand_bridge.py                       # /dev/ttyACM0 -> 0.0.0.0:9031
    python3 aihand_bridge.py --device /dev/ttyACM1 --port 9031
    python3 aihand_bridge.py --list                # what serial devices exist

STDLIB ONLY, on purpose: it has to run on whatever Python the vendor image ships
with, and pyserial may not be installed there. A USB CDC device ignores baud
rate entirely, so opening the device node as a binary file is sufficient and
avoids needing termios at all.

ONE WAY ONLY. The AiHand has no feedback of any kind, so there is nothing to
read back; the bridge forwards laptop-to-hand and nothing else. If a future hand
does report state, that is a new protocol and a new bridge, not a flag here.

Serves ONE client at a time, deliberately: two processes interleaving frames
into one servo controller produces motion neither of them asked for.
"""
import argparse
import glob
import socket
import sys


def list_devices() -> list[str]:
    return sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))


def serve(device: str, host: str, port: int) -> int:
    try:
        # Unbuffered binary write: a servo command must hit the wire when it is
        # sent, not when a buffer happens to flush.
        tty = open(device, "wb", buffering=0)
    except OSError as exc:
        print(f"cannot open {device}: {exc}", file=sys.stderr)
        found = list_devices()
        print(f"serial devices present: {found or 'none'}", file=sys.stderr)
        if not found:
            print("nothing is plugged in, or brltty grabbed the port "
                  "(sudo systemctl stop brltty)", file=sys.stderr)
        return 1

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(1)
    print(f"aihand bridge: {device} <- {host}:{port} (one client at a time)",
          flush=True)

    try:
        while True:
            conn, addr = srv.accept()
            print(f"client {addr[0]}:{addr[1]} connected", flush=True)
            total = 0
            try:
                while True:
                    chunk = conn.recv(4096)
                    if not chunk:
                        break
                    tty.write(chunk)
                    total += len(chunk)
                    print(f"  -> {len(chunk)}B {chunk!r}", flush=True)
            except OSError as exc:
                print(f"  client error: {exc}", flush=True)
            finally:
                conn.close()
                print(f"client gone ({total}B forwarded)", flush=True)
    except KeyboardInterrupt:
        print("\nstopped", flush=True)
        return 0
    finally:
        srv.close()
        tty.close()


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--device", default="/dev/ttyACM0",
                   help="the micro:bit's serial device (default /dev/ttyACM0)")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=9031)
    p.add_argument("--list", action="store_true",
                   help="list serial devices and exit")
    args = p.parse_args()

    if args.list:
        found = list_devices()
        print("\n".join(found) if found else "no serial devices found")
        return 0
    return serve(args.device, args.host, args.port)


if __name__ == "__main__":
    raise SystemExit(main())
