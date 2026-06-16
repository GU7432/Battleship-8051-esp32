"""Pure game-logic layer: no sockets here, only Battleship rules + board encoding.

Board conventions
-----------------
* Strict 8x8 grid.  x = column (0..7, left->right), y = row (0..7, top->bottom).
* A "board" sent to the matrix is encoded as two 8-byte bit planes:
      bytes 0..7  = GREEN plane, one byte per row, bit (1<<x) = column x
      bytes 8..15 = RED plane,   same layout
  The 8051 just copies these 16 bytes straight into its frame buffer.
* The full S_STATE payload is 32 bytes = OWN board (16) followed by ENEMY board (16).
"""

from __future__ import annotations

import random

from . import protocol as P

# Classic fleet for an 8x8 board: ships of these lengths (sum = 14 cells).
FLEET_SIZES = (4, 3, 3, 2, 2)


def popcount(v: int) -> int:
    return bin(v & 0xFF).count("1")


def random_fleet(fleet_sizes=FLEET_SIZES) -> list[int]:
    """Randomly place a fleet of straight ships on the 8x8 grid (no overlap).

    Returns 8 row bitmasks (bit (1<<x) set => ship cell at column x, that row).
    """
    grid = [[0] * 8 for _ in range(8)]
    for size in fleet_sizes:
        for _ in range(2000):  # retry until this ship fits
            horizontal = random.random() < 0.5
            if horizontal:
                x = random.randint(0, 8 - size)
                y = random.randint(0, 7)
                cells = [(x + i, y) for i in range(size)]
            else:
                x = random.randint(0, 7)
                y = random.randint(0, 8 - size)
                cells = [(x, y + i) for i in range(size)]
            if all(grid[cy][cx] == 0 for cx, cy in cells):
                for cx, cy in cells:
                    grid[cy][cx] = 1
                break
    return [sum(1 << x for x in range(8) if grid[y][x]) for y in range(8)]


class Player:
    """One seat in the game."""

    def __init__(self, pid: int, transport) -> None:
        self.pid = pid
        self.transport = transport          # opaque handle used by the server I/O layer
        self.ships = [0] * 8                 # 8 row bitmasks: bit (1<<x) set => ship at (x,row)
        self.shots: set[tuple[int, int]] = set()  # cells THIS player has fired at the enemy
        self.placed = False

    def ship_at(self, x: int, y: int) -> bool:
        return bool((self.ships[y] >> x) & 1)

    @property
    def ship_cells(self) -> int:
        return sum(popcount(row) for row in self.ships)


def _planes(states: list[list[int]]) -> bytes:
    """Render an 8x8 grid of logical states into a 16-byte (green+red) bit plane block."""
    green = bytearray(8)
    red = bytearray(8)
    for y in range(8):
        for x in range(8):
            s = states[y][x]
            if s in (P.SHIP, P.MISS):   # green lit for ship and miss
                green[y] |= 1 << x
            if s in (P.HIT, P.MISS):    # red lit for hit and miss (miss = both = amber)
                red[y] |= 1 << x
    return bytes(green) + bytes(red)


class Game:
    """Holds up to two players and the turn/win state."""

    def __init__(self, required_ship_cells: int = 14) -> None:
        self.players: list[Player] = []
        self.required_ship_cells = required_ship_cells
        self.turn = 0          # pid whose turn it is to fire
        self.started = False
        self.over = False
        self.winner: int | None = None

    # ---- seat management ------------------------------------------------
    @property
    def full(self) -> bool:
        return len(self.players) >= 2

    def add_player(self, transport) -> Player:
        pid = len(self.players)
        p = Player(pid, transport)
        self.players.append(p)
        return p

    def opponent(self, player: Player) -> Player:
        return self.players[1 - player.pid]

    def reset(self) -> None:
        self.players.clear()
        self.turn = 0
        self.started = False
        self.over = False
        self.winner = None

    # ---- rules ----------------------------------------------------------
    def place(self, player: Player, rows: bytes | None = None) -> bool:
        """Place a player's fleet.

        * rows is None (normal play): the server assigns a fresh RANDOM fleet,
          so every match has a different layout. Idempotent -- a player who
          re-sends C_PLACE (e.g. resend-until-started) keeps their first fleet.
        * rows given (tests): validate and use that explicit 8-byte layout.
        Returns True if accepted.
        """
        if player.placed:
            return True  # already has a fleet; don't re-roll on a resend

        if rows is None:
            player.ships = random_fleet()
        else:
            if len(rows) != 8:
                return False
            if sum(popcount(b) for b in rows) != self.required_ship_cells:
                return False
            player.ships = list(rows)

        player.placed = True
        if all(p.placed for p in self.players) and self.full:
            self.started = True
            self.turn = 0
        return True

    def fire(self, player: Player, x: int, y: int) -> int:
        """Register a shot. Returns OUT_HIT / OUT_MISS. Caller must validate turn first."""
        opp = self.opponent(player)
        player.shots.add((x, y))
        return P.OUT_HIT if opp.ship_at(x, y) else P.OUT_MISS

    def is_defeated(self, player: Player) -> bool:
        """True when every ship cell of `player` has been hit by the opponent."""
        opp = self.opponent(player)
        if player.ship_cells == 0:
            return False
        hits = sum(1 for (x, y) in opp.shots if player.ship_at(x, y))
        return hits >= player.ship_cells

    # ---- display encoding ----------------------------------------------
    def _own_states(self, player: Player) -> list[list[int]]:
        """The player's OWN board: their ships + the enemy's shots landing on them."""
        opp = self.opponent(player)
        g = [[P.EMPTY] * 8 for _ in range(8)]
        for y in range(8):
            for x in range(8):
                if player.ship_at(x, y):
                    g[y][x] = P.SHIP
        for (x, y) in opp.shots:
            g[y][x] = P.HIT if player.ship_at(x, y) else P.MISS
        return g

    def _enemy_states(self, player: Player) -> list[list[int]]:
        """The player's view of the ENEMY board: only their own shots are revealed."""
        opp = self.opponent(player)
        g = [[P.EMPTY] * 8 for _ in range(8)]
        for (x, y) in player.shots:
            g[y][x] = P.HIT if opp.ship_at(x, y) else P.MISS
        return g

    def state_payload(self, player: Player) -> bytes:
        """32-byte S_STATE payload for `player`: own board (16) + enemy board (16)."""
        return _planes(self._own_states(player)) + _planes(self._enemy_states(player))
