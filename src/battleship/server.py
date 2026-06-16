"""Asyncio TCP server that drives the Battleship game.

Run with:
    uv run battleship-server --host 0.0.0.0 --port 9000

The server keeps a single :class:`~battleship.game.Game` (one 1-v-1 match at a
time). The 8051 nodes never need to track game state -- the server pushes a full
32-byte S_STATE after every change, so each node just renders whatever arrives
and sends C_FIRE when it is its turn.
"""

from __future__ import annotations

import argparse
import asyncio
import logging

from . import protocol as P
from .game import Game, Player

log = logging.getLogger("battleship")


# --------------------------------------------------------------------------
# Frame I/O over an asyncio stream
# --------------------------------------------------------------------------
async def read_frame(reader: asyncio.StreamReader) -> tuple[int, bytes]:
    """Read one validated frame. Resynchronises on the SYNC byte.

    Raises asyncio.IncompleteReadError on EOF and ValueError on a bad checksum.
    """
    # Hunt for the SYNC byte (lets us recover from a corrupted/partial frame).
    while True:
        b = await reader.readexactly(1)
        if b[0] == P.SYNC:
            break
    header = await reader.readexactly(2)
    type_, length = header[0], header[1]
    payload = await reader.readexactly(length) if length else b""
    chk = (await reader.readexactly(1))[0]
    if chk != P.checksum(type_, payload):
        raise ValueError(f"bad checksum on type=0x{type_:02X}")
    return type_, payload


async def send(writer: asyncio.StreamWriter, type_: int, payload: bytes = b"") -> None:
    writer.write(P.build_frame(type_, payload))
    await writer.drain()


# --------------------------------------------------------------------------
# Server
# --------------------------------------------------------------------------
class BattleshipServer:
    def __init__(self, required_ship_cells: int = 14) -> None:
        self.game = Game(required_ship_cells=required_ship_cells)
        self.lock = asyncio.Lock()  # serialise all state mutations

    # ---- broadcast helpers (player.transport is the StreamWriter) -------
    async def push_state_all(self) -> None:
        for p in self.game.players:
            await send(p.transport, P.S_STATE, self.game.state_payload(p))

    async def push_turn_all(self) -> None:
        for p in self.game.players:
            yours = 1 if (self.game.turn == p.pid and not self.game.over) else 0
            await send(p.transport, P.S_TURN, bytes([yours]))

    # ---- per-connection handler -----------------------------------------
    async def handle(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        peer = writer.get_extra_info("peername")

        async with self.lock:
            if self.game.full:
                log.info("rejecting %s: game full", peer)
                await send(writer, P.S_ERROR, bytes([P.E_BUSY]))
                writer.close()
                return
            player = self.game.add_player(writer)

        log.info("player %d connected from %s", player.pid, peer)
        await send(writer, P.S_WELCOME, bytes([player.pid]))

        try:
            while True:
                try:
                    type_, payload = await read_frame(reader)
                except ValueError as exc:
                    # Corrupted frame on a noisy UART link: drop it and resync
                    # to the next SYNC byte. Do NOT tear down the connection --
                    # that would reset the whole game on a single bad byte.
                    log.warning("player %d: %s (frame dropped, link kept)", player.pid, exc)
                    continue
                async with self.lock:
                    await self.dispatch(player, type_, payload)
        except (asyncio.IncompleteReadError, ConnectionResetError):
            log.info("player %d disconnected", player.pid)
        finally:
            await self.on_disconnect(player)
            writer.close()

    # ---- message dispatch ----------------------------------------------
    async def dispatch(self, player: Player, type_: int, payload: bytes) -> None:
        log.debug("player %d RX type=0x%02X len=%d payload=%s",
                  player.pid, type_, len(payload), payload.hex(" "))
        if type_ == P.C_HELLO:
            if not self.game.full:
                await send(player.transport, P.S_WAIT)
            elif self.game.started:
                # The node rebooted mid-game (its TCP socket on the ESP32 stayed
                # up): re-sync its display instead of restarting the match.
                await send(player.transport, P.S_STATE, self.game.state_payload(player))
                yours = 1 if (self.game.turn == player.pid and not self.game.over) else 0
                await send(player.transport, P.S_TURN, bytes([yours]))
            return

        if type_ == P.C_PLACE:
            await self.handle_place(player, payload)
            return

        if type_ == P.C_FIRE:
            await self.handle_fire(player, payload)
            return

        if type_ == P.C_RESET:
            await self.handle_reset(player)
            return

        log.warning("player %d: unknown type 0x%02X", player.pid, type_)
        await send(player.transport, P.S_ERROR, bytes([P.E_STATE]))

    async def handle_place(self, player: Player, payload: bytes) -> None:
        if self.game.started:
            await send(player.transport, P.S_ERROR, bytes([P.E_STATE]))
            return
        # The server assigns a fresh RANDOM fleet each match; the node's C_PLACE
        # payload is just a "ready" signal (its contents are ignored).
        already = player.placed
        self.game.place(player)
        if not already:
            log.info("player %d placed (random fleet, cells=%d)", player.pid, player.ship_cells)

        if self.game.started:  # both placed -> kick off the match
            log.info("both players placed; game starting")
            for p in self.game.players:
                await send(p.transport, P.S_START, bytes([1 if self.game.turn == p.pid else 0]))
            await self.push_state_all()
            await self.push_turn_all()
        else:
            await send(player.transport, P.S_WAIT)

    async def handle_fire(self, player: Player, payload: bytes) -> None:
        if not self.game.started or self.game.over:
            await send(player.transport, P.S_ERROR, bytes([P.E_STATE]))
            return
        if self.game.turn != player.pid:
            await send(player.transport, P.S_ERROR, bytes([P.E_NOTURN]))
            return
        if len(payload) != 1:
            await send(player.transport, P.S_ERROR, bytes([P.E_STATE]))
            return

        x, y = P.unpack_coord(payload[0])
        if x > 7 or y > 7:
            await send(player.transport, P.S_ERROR, bytes([P.E_STATE]))
            return
        if (x, y) in player.shots:
            await send(player.transport, P.S_ERROR, bytes([P.E_DUP]))
            return

        outcome = self.game.fire(player, x, y)
        await send(player.transport, P.S_RESULT, bytes([P.pack_coord(x, y), outcome]))

        opponent = self.game.opponent(player)
        if self.game.is_defeated(opponent):
            self.game.over = True
            self.game.winner = player.pid
            await self.push_state_all()
            await send(player.transport, P.S_OVER, bytes([1]))
            await send(opponent.transport, P.S_OVER, bytes([0]))
            log.info("player %d wins", player.pid)
            return

        # Turn alternates after every shot (set NEXT line to keep turn on hit
        # if you prefer the "fire again on hit" variant).
        self.game.turn = opponent.pid
        await self.push_state_all()
        await self.push_turn_all()

    async def handle_reset(self, player: Player) -> None:
        log.info("player %d requested reset", player.pid)
        for p in list(self.game.players):
            if p is not player:
                try:
                    await send(p.transport, P.S_OVER, bytes([1]))
                except Exception:
                    pass
        self.game.reset()

    async def on_disconnect(self, player: Player) -> None:
        if player not in self.game.players:
            return
        # Tell the survivor the match is dead, then wipe the game for a rematch.
        for p in self.game.players:
            if p is not player:
                try:
                    await send(p.transport, P.S_ERROR, bytes([P.E_STATE]))
                except Exception:
                    pass
        self.game.reset()


# --------------------------------------------------------------------------
# Entry point
# --------------------------------------------------------------------------
async def serve(host: str, port: int, ship_cells: int) -> None:
    server = BattleshipServer(required_ship_cells=ship_cells)
    srv = await asyncio.start_server(server.handle, host, port)
    addrs = ", ".join(str(s.getsockname()) for s in srv.sockets)
    log.info("Battleship server listening on %s (ship cells required: %d)", addrs, ship_cells)
    async with srv:
        await srv.serve_forever()


def main() -> None:
    parser = argparse.ArgumentParser(description="Battleship byte-protocol TCP server")
    parser.add_argument("--host", default="0.0.0.0", help="bind address (default 0.0.0.0)")
    parser.add_argument("--port", type=int, default=9000, help="TCP port (default 9000)")
    parser.add_argument(
        "--ship-cells",
        type=int,
        default=14,
        help="exact number of ship cells each player must place (default 14)",
    )
    parser.add_argument("-v", "--verbose", action="store_true", help="debug logging")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )
    try:
        asyncio.run(serve(args.host, args.port, args.ship_cells))
    except KeyboardInterrupt:
        log.info("shutting down")


if __name__ == "__main__":
    main()
