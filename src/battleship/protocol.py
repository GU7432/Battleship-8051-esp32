"""Byte-level wire protocol for the Battleship server.

Design goals
------------
* Zero JSON / zero ASCII parsing. Everything is fixed binary so an 8051 can
  decode it with a tiny state machine and a couple of array writes.
* Self-framing stream protocol (TCP is a byte stream, not message-oriented):
  every message is delimited by a SYNC byte, a TYPE, a LENGTH and a trailing
  XOR checksum. The 8051 can resynchronise on the SYNC byte if it ever loses
  a byte over UART.

Frame layout (on the wire, MSB first is irrelevant -- everything is bytes)
--------------------------------------------------------------------------
    +------+------+--------+-------------------+----------+
    | SYNC | TYPE | LENGTH |  PAYLOAD (LENGTH) | CHECKSUM |
    +------+------+--------+-------------------+----------+
      0xAA   1B     1B        0..255 bytes        1B

    CHECKSUM = TYPE XOR LENGTH XOR payload[0] XOR ... XOR payload[LENGTH-1]

A whole frame is therefore  4 + LENGTH  bytes long.
"""

# --------------------------------------------------------------------------
# Framing
# --------------------------------------------------------------------------
SYNC = 0xAA  # start-of-frame marker

# --------------------------------------------------------------------------
# Message types : client (8051/ESP32) -> server   (low bytes, bit7 = 0)
# --------------------------------------------------------------------------
C_HELLO = 0x01  # payload: none.          "I am here, assign me a seat."
C_PLACE = 0x02  # payload: 8 bytes.       Ship layout, one bitmask byte per row.
C_FIRE = 0x03   # payload: 1 byte coord.  Fire at packed (x,y).
C_RESET = 0x0F  # payload: none.          Forfeit / restart.

# --------------------------------------------------------------------------
# Message types : server -> client (8051/ESP32)   (high bytes, bit7 = 1)
# --------------------------------------------------------------------------
S_WELCOME = 0x81  # payload: 1 byte  -> your player id (0 or 1)
S_WAIT = 0x82     # payload: none    -> waiting for the opponent
S_START = 0x83    # payload: 1 byte  -> game started; 1 = you move first, 0 = wait
S_STATE = 0x84    # payload: 32 bytes-> full display: own board(16) + enemy board(16)
S_TURN = 0x85     # payload: 1 byte  -> 1 = it is your turn to fire, 0 = opponent's turn
S_RESULT = 0x86   # payload: 2 bytes -> [packed coord, outcome] for the shot YOU fired
S_OVER = 0x87     # payload: 1 byte  -> 1 = you win, 0 = you lose
S_ERROR = 0x8F    # payload: 1 byte  -> error code (see E_* below)

# --------------------------------------------------------------------------
# Logical cell states (2 bits of meaning, rendered onto the bi-color matrix)
# --------------------------------------------------------------------------
#   state   green  red   LED colour     meaning
#   -----   -----  ---   -----------    -------------------------------------
EMPTY = 0   # 0      0    off           open water / unknown
SHIP = 1    # 1      0    green         your intact ship
MISS = 2    # 1      1    amber/orange  a shot that hit water
HIT = 3     # 0      1    red           a ship cell that was hit

# Outcome codes used in S_RESULT
OUT_MISS = 0
OUT_HIT = 1

# Error codes used in S_ERROR
E_BUSY = 1      # server already has two players
E_BADPLACE = 2  # illegal ship layout (wrong cell count / wrong length)
E_NOTURN = 3    # you fired but it is not your turn
E_DUP = 4       # you fired at a cell you already fired at
E_STATE = 5     # message not valid in the current game phase


def checksum(type_: int, payload: bytes) -> int:
    """XOR checksum over TYPE, LENGTH and every payload byte."""
    c = type_ ^ len(payload)
    for b in payload:
        c ^= b
    return c & 0xFF


def build_frame(type_: int, payload: bytes = b"") -> bytes:
    """Wrap a payload into a complete on-the-wire frame."""
    if len(payload) > 255:
        raise ValueError("payload too long")
    return bytes([SYNC, type_, len(payload)]) + payload + bytes([checksum(type_, payload)])


def pack_coord(x: int, y: int) -> int:
    """Pack a board coordinate into a single byte: high nibble = X (col), low = Y (row)."""
    return ((x & 0x0F) << 4) | (y & 0x0F)


def unpack_coord(b: int) -> tuple[int, int]:
    """Inverse of :func:`pack_coord` -> (x col, y row)."""
    return (b >> 4) & 0x0F, b & 0x0F
