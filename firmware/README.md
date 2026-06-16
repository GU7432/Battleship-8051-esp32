# Battleship Node Firmware

Two files, one per chip in a player node. The pair forms a transparent path:

```
Python server  <--TCP-->  ESP32 (Wi-Fi bridge)  <--UART 9600 8N1-->  8051  -->  8x8 bi-color matrix
```

The ESP32 is *dumb* (pure byte pipe). All protocol parsing, framing, rendering,
and input live on the 8051.

```
firmware/
├── esp32/main.py            MicroPython: Wi-Fi + raw TCP <-> UART2 passthrough
└── 8051/battleship_8051.c   Keil C51: UART parser, matrix multiplex, button input
```

---

## ESP32 (`esp32/main.py`, MicroPython)

Edit the **USER CONFIG** block at the top of `main.py`:

| Variable | Set to |
|----------|--------|
| `WIFI_SSID` / `WIFI_PASS` | your access point |
| `SERVER_HOST` / `SERVER_PORT` | the Python server (`9000` default) |
| `UART_BAUD` | **9600** — must match the 8051 |
| `UART_TX` / `UART_RX` | UART2 pins (default GPIO17 / GPIO16) |
| `LED_PIN` | onboard LED (lit while the TCP link is up), or `None` |

Notes:
- TCP `TCP_NODELAY` is set so 4–36 byte frames go out instantly (Nagle off).
- Non-blocking + `select.poll()` on both the socket and the UART → low latency,
  no dropped frames, auto-reconnect on Wi-Fi/TCP loss.
- **Level-shift** the 8051-TX → ESP32-RX line (8051 is 5 V, ESP32 RX is 3.3 V only).

Flash & deploy:
1. Install MicroPython once (erase + write the ESP32 `.bin`):
   ```bash
   esptool.py --chip esp32 erase_flash
   esptool.py --chip esp32 --baud 460800 write_flash 0x1000 esp32-xxxx.bin
   ```
2. Copy `main.py` to the board with **mpremote** or **Thonny**:
   ```bash
   mpremote connect auto fs cp main.py :main.py
   ```
3. It runs automatically on reset. Watch logs over the USB REPL (115200):
   ```bash
   mpremote connect auto repl
   ```

---

## 8051 (`8051/battleship_8051.c`)

Target **STC90C516RD+**, **11.0592 MHz** crystal, Keil uVision2 / C51. Classic
SFR + `sbit` style, no C99.

### Pin map (change the `sbit` lines to match your board)

Topology matches `firmware/8051/Example`: **3× 74HC595 in two chains**. Chain A
(on P1) is **two 595s daisy-chained** = 16 column bits (8 red + 8 green); Chain B
(on P2) is one 595 for row select.

| Signal | Pin | Goes to |
|--------|-----|---------|
| `SR_DATA`   | P1.2 | Chain A DS (column data, 2 chips chained) |
| `SR_CLK`    | P1.1 | Chain A SHCP (shift clock) |
| `SR_LATCH`  | P1.0 | Chain A STCP (latch) |
| `SR_DATA_B` | P2.0 | Chain B DS (row select) |
| `SR_CLK_B`  | P2.1 | Chain B SHCP |
| `SR_LATCH_B`| P2.2 | Chain B STCP |
| `BTN_UP/DOWN/LEFT/RIGHT/FIRE` | P2.3–P2.7 | buttons to GND (active low) |
| `BTN_VIEW`  | P3.2 | button to GND (active low) |
| UART RXD / TXD | P3.0 / P3.1 | ESP32 UART2 |

Each row, `display_scan()` blanks the columns, selects the row on Chain B, then
sends **two colour bytes** (red, green) into Chain A and latches — both colours lit
per row, 8-row scan ≈ 125 Hz. This board is **column active-low, row active-high**
(`COL_INVERT 1`, `ROW_INVERT 0`). If red/green look swapped, exchange the two
`shift_byte(r)` / `shift_byte(g)` lines in `display_scan()`.

### What it does
- **UART RX ISR** runs the framing state machine `SYNC|TYPE|LEN|PAYLOAD|CHK`,
  verifies the XOR checksum, and resyncs on `0xAA` after any error.
- **Timer0 ISR** multiplexes the matrix from `fb_green[8]` / `fb_red[8]` at ~125 Hz
  (8-row scan, both colours lit per row via the 16-bit column chain).
- **Timer1** is the 9600-baud generator; **TX is an interrupt-driven ring buffer**.
- On `S_STATE` (32 B) it stores both boards; the **VIEW** button toggles which
  board is shown (own = bytes 0–15, enemy = bytes 16–31).
- **FIRE** packs the cursor into one byte and sends a `C_FIRE` frame.
- It auto-sends `C_HELLO` + `C_PLACE` (a 14-cell default fleet) every ~1 s until
  the server answers `S_WELCOME`, so connection timing never matters.

### Build in Keil uVision2
1. New project → device **STC90C516RD+** (or any 8052 core; pick "Atmel AT89C52"
   if STC isn't listed — register-compatible for this code).
2. Add `battleship_8051.c`, set Xtal = **11.0592 MHz**.
3. Build, then flash the HEX with STC-ISP.

> Reminder: the server's 32-byte `S_STATE` is **two boards** (own then enemy),
> each being 8 green rows + 8 red rows — not "16 green + 16 red". The code is
> written to that real layout.
