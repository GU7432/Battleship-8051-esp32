"""Tiny CLI test client that emulates one 8051+ESP32 node over plain TCP.

Run the server in one terminal, then two of these in two others:

    uv run battleship-server
    uv run battleship-client            # player A
    uv run battleship-client            # player B

Each client auto-places a fixed fleet, then lets you type fire coordinates as
"x y" (0..7). It prints the boards it receives so you can watch the match.
"""

from __future__ import annotations

import argparse
import socket
import threading

from . import protocol as P

# A fixed legal fleet = 14 cells (sizes 4,3,3,2,2), kept clear of edges/overlap.
DEFAULT_FLEET = bytes(
    [
        0b00001111,  # row 0: ship of 4 at x=0..3
        0b00000000,  # row 1
        0b01110000,  # row 2: ship of 3 at x=4..6
        0b00000000,  # row 3
        0b00000111,  # row 4: ship of 3 at x=0..2
        0b00000000,  # row 5
        0b11000000,  # row 6: ship of 2 at x=6..7
        0b00011000,  # row 7: ship of 2 at x=3..4
    ]
)


def recv_frame(sock: socket.socket) -> tuple[int, bytes]:
    def rd(n: int) -> bytes:
        buf = b""
        while len(buf) < n:
            chunk = sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("server closed")
            buf += chunk
        return buf

    while rd(1)[0] != P.SYNC:
        pass
    type_, length = rd(2)
    payload = rd(length) if length else b""
    rd(1)  # checksum (trusted for the demo)
    return type_, payload


def show_board(label: str, planes: bytes) -> None:
    green, red = planes[:8], planes[8:]
    print(f"  {label}:")
    for y in range(8):
        line = []
        for x in range(8):
            g = (green[y] >> x) & 1
            r = (red[y] >> x) & 1
            line.append({(0, 0): ".", (1, 0): "S", (0, 1): "X", (1, 1): "o"}[(g, r)])
        print("    " + " ".join(line))


def reader_loop(sock: socket.socket) -> None:
    try:
        while True:
            type_, payload = recv_frame(sock)
            if type_ == P.S_WELCOME:
                print(f"[WELCOME] you are player {payload[0]}")
            elif type_ == P.S_WAIT:
                print("[WAIT] waiting for opponent / placement")
            elif type_ == P.S_START:
                print(f"[START] you move first = {payload[0]}")
            elif type_ == P.S_STATE:
                print("[STATE]  (S=ship  X=hit  o=miss  .=empty)")
                show_board("OWN  ", payload[:16])
                show_board("ENEMY", payload[16:])
            elif type_ == P.S_TURN:
                print("[TURN] >>> YOUR TURN <<<" if payload[0] else "[TURN] opponent's turn")
            elif type_ == P.S_RESULT:
                x, y = P.unpack_coord(payload[0])
                print(f"[RESULT] your shot ({x},{y}) -> {'HIT' if payload[1] else 'MISS'}")
            elif type_ == P.S_OVER:
                print("[OVER] *** YOU WIN ***" if payload[0] else "[OVER] you lose")
            elif type_ == P.S_ERROR:
                print(f"[ERROR] code {payload[0]}")
            else:
                print(f"[?] type=0x{type_:02X} payload={payload.hex()}")
    except (ConnectionError, OSError):
        print("[disconnected]")


def main() -> None:
    ap = argparse.ArgumentParser(description="Battleship demo client")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=9000)
    args = ap.parse_args()

    sock = socket.create_connection((args.host, args.port))
    threading.Thread(target=reader_loop, args=(sock,), daemon=True).start()

    sock.sendall(P.build_frame(P.C_HELLO))
    sock.sendall(P.build_frame(P.C_PLACE, DEFAULT_FLEET))
    print("Placed fleet. Type 'x y' to fire (0..7), or 'q' to quit.")

    try:
        while True:
            line = input().strip()
            if line in ("q", "quit", "exit"):
                break
            try:
                x, y = (int(v) for v in line.split())
            except ValueError:
                print("enter two numbers, e.g. '3 4'")
                continue
            sock.sendall(P.build_frame(P.C_FIRE, bytes([P.pack_coord(x, y)])))
    except (EOFError, KeyboardInterrupt):
        pass
    finally:
        sock.close()


if __name__ == "__main__":
    main()
