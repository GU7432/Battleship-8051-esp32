"""Tests for the Battleship server.

  * Unit checks of the pure game logic (deterministic, explicit placement).
  * random_fleet() validity (correct cell count => straight ships, no overlap).
  * End-to-end socket test with the now-default RANDOM placement: two emulated
    nodes connect, the server assigns random fleets, both fire the whole board,
    and exactly one wins.

Run with:  uv run python tests/test_e2e.py
"""

import asyncio

from battleship import protocol as P
from battleship.game import FLEET_SIZES, Game, popcount, random_fleet
from battleship.server import BattleshipServer, read_frame, send


# --------------------------------------------------------------------------
# 1. Pure game-logic unit checks (deterministic)
# --------------------------------------------------------------------------
def test_game_logic():
    g = Game(required_ship_cells=2)
    p0 = g.add_player(None)
    p1 = g.add_player(None)

    # Explicit 2-cell fleets at (0,0),(1,0).
    assert g.place(p0, bytes([0x03, 0, 0, 0, 0, 0, 0, 0]))
    assert not g.started, "should not start until both placed"
    assert g.place(p1, bytes([0x03, 0, 0, 0, 0, 0, 0, 0]))
    assert g.started and g.turn == 0

    # Re-placing is idempotent (does not re-roll / error).
    assert g.place(p0, bytes([0xFF, 0, 0, 0, 0, 0, 0, 0]))
    assert g.players[0].ship_cells == 2

    assert g.fire(p0, 0, 0) == P.OUT_HIT
    assert not g.is_defeated(p1)
    assert g.fire(p0, 5, 5) == P.OUT_MISS
    assert g.fire(p0, 1, 0) == P.OUT_HIT
    assert g.is_defeated(p1)

    assert len(g.state_payload(p0)) == 32
    print("UNIT game-logic OK")


def test_random_fleet():
    for _ in range(300):
        rows = random_fleet()
        assert len(rows) == 8
        # Exactly the fleet's cell count => every ship placed with no overlap.
        assert sum(popcount(r) for r in rows) == sum(FLEET_SIZES)
    print("UNIT random_fleet OK (cells =", sum(FLEET_SIZES), ")")


# --------------------------------------------------------------------------
# 2. End-to-end socket test (random placement)
# --------------------------------------------------------------------------
async def play_client(host, port, fires, name, results, seen):
    reader, writer = await asyncio.open_connection(host, port)
    await send(writer, P.C_HELLO)
    await send(writer, P.C_PLACE, bytes(8))   # payload ignored; server randomizes
    fire_iter = iter(fires)
    try:
        while True:
            t, pl = await asyncio.wait_for(read_frame(reader), timeout=5)
            if t == P.S_STATE:
                seen["state_len"] = len(pl)
            elif t == P.S_RESULT and len(pl) == 2 and pl[1] == P.OUT_HIT:
                seen["hit"] = True
            elif t == P.S_TURN and pl and pl[0] == 1:
                nxt = next(fire_iter, None)
                if nxt is not None:
                    await send(writer, P.C_FIRE, bytes([P.pack_coord(*nxt)]))
            elif t == P.S_OVER:
                results[name] = "WIN" if (pl and pl[0]) else "LOSE"
                return
    except (asyncio.IncompleteReadError, asyncio.TimeoutError):
        results.setdefault(name, "?")
    finally:
        writer.close()


async def integration():
    server = BattleshipServer()
    srv = await asyncio.start_server(server.handle, "127.0.0.1", 0)
    host, port = srv.sockets[0].getsockname()
    results, seen = {}, {}
    all_cells = [(x, y) for y in range(8) for x in range(8)]  # whole board

    async with srv:
        c0 = asyncio.create_task(play_client(host, port, all_cells, "p0", results, seen))
        await asyncio.sleep(0.1)  # deterministic seat order
        c1 = asyncio.create_task(play_client(host, port, all_cells, "p1", results, seen))
        await asyncio.wait_for(asyncio.gather(c0, c1), timeout=30)

    assert seen.get("state_len") == 32, f"S_STATE must be 32 bytes, got {seen.get('state_len')}"
    assert seen.get("hit"), "no hit was ever recorded"
    wins = [k for k, v in results.items() if v == "WIN"]
    loses = [k for k, v in results.items() if v == "LOSE"]
    assert len(wins) == 1 and len(loses) == 1, f"expected one win + one lose, got {results}"
    print("INTEGRATION OK:", results)


def main():
    test_game_logic()
    test_random_fleet()
    asyncio.run(integration())
    print("ALL TESTS PASSED")


if __name__ == "__main__":
    main()
