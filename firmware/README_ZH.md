# 海戰棋節點韌體

兩個檔案，一個晶片一份。兩者組成一條透明資料路徑：

```
Python 伺服器  <--TCP-->  ESP32（Wi-Fi 橋接）  <--UART 9600 8N1-->  8051  -->  8x8 雙色點陣
```

ESP32 是*啞的*（純 byte 管道）。所有協議解析、分幀、畫面繪製與輸入處理
全都在 8051 上。

```
firmware/
├── esp32/main.py            MicroPython：Wi-Fi + raw TCP <-> UART2 透傳
└── 8051/battleship_8051.c   Keil C51：UART 解析、點陣多工掃描、按鈕輸入
```

---

## ESP32（`esp32/main.py`，MicroPython）

編輯 `main.py` 最上方的 **USER CONFIG** 區塊：

| 變數 | 設定為 |
|------|--------|
| `WIFI_SSID` / `WIFI_PASS` | 你的無線基地台 |
| `SERVER_HOST` / `SERVER_PORT` | Python 伺服器（預設 `9000`） |
| `UART_BAUD` | **9600** — 必須與 8051 一致 |
| `UART_TX` / `UART_RX` | UART2 腳位（預設 GPIO17 / GPIO16） |
| `LED_PIN` | 板載 LED（TCP 連線中時恆亮），沒有就設 `None` |

說明：
- 已設定 TCP `TCP_NODELAY` 關閉 Nagle，讓 4–36 byte 的小幀**立即送出**。
- 對 socket 與 UART 都採非阻塞 + `select.poll()` → 低延遲、不掉幀，
  Wi-Fi/TCP 斷線時自動重連。
- **務必電平轉換** 8051-TX → ESP32-RX 這條線（8051 是 5V，ESP32 的 RX 只能吃 3.3V）。

燒錄與部署：
1. 先燒一次 MicroPython 韌體（抹除 + 寫入 ESP32 `.bin`）：
   ```bash
   esptool.py --chip esp32 erase_flash
   esptool.py --chip esp32 --baud 460800 write_flash 0x1000 esp32-xxxx.bin
   ```
2. 用 **mpremote** 或 **Thonny** 把 `main.py` 複製到板子：
   ```bash
   mpremote connect auto fs cp main.py :main.py
   ```
3. 重置後自動執行。透過 USB REPL（115200）觀看日誌：
   ```bash
   mpremote connect auto repl
   ```

---

## 8051（`8051/battleship_8051.c`）

目標 **STC90C516RD+**、**11.0592 MHz** 晶振、Keil uVision2 / C51。
傳統 SFR + `sbit` 寫法，不使用 C99。

### 腳位對應（改 `sbit` 那幾行以符合你的板子）

拓撲與 `firmware/8051/Example` 一致：**3 顆 74HC595、兩條鏈**。Chain A（P1）為
**兩顆 595 串接** = 16 bit 欄位（8 紅 + 8 綠）；Chain B（P2）為一顆 595 做列選。

| 訊號 | 腳位 | 接到 |
|------|------|------|
| `SR_DATA`   | P1.2 | Chain A DS（欄位資料，2 顆串接） |
| `SR_CLK`    | P1.1 | Chain A SHCP（移位時脈） |
| `SR_LATCH`  | P1.0 | Chain A STCP（鎖存） |
| `SR_DATA_B` | P2.0 | Chain B DS（列選） |
| `SR_CLK_B`  | P2.1 | Chain B SHCP |
| `SR_LATCH_B`| P2.2 | Chain B STCP |
| `BTN_UP/DOWN/LEFT/RIGHT/FIRE` | P2.3–P2.7 | 按鈕接 GND（低電位有效） |
| `BTN_VIEW`  | P3.2 | 按鈕接 GND（低電位有效） |
| UART RXD / TXD | P3.0 / P3.1 | ESP32 UART2 |

每一列 `display_scan()` 會先消隱欄位、用 Chain B 選列，再把**兩個顏色 byte**
（紅、綠）送進 Chain A 並鎖存——**該列紅綠同時點亮**，8 列掃描約 125 Hz。
本板為**欄位低電位點亮、列選高電位選通**（`COL_INVERT 1`、`ROW_INVERT 0`）。
若紅綠顯示相反，對調 `display_scan()` 中 `shift_byte(r)` / `shift_byte(g)` 兩行即可。

### 它做了什麼
- **UART 接收 ISR** 跑 `SYNC|TYPE|LEN|PAYLOAD|CHK` 分幀狀態機，
  驗證 XOR 校驗，任何錯誤後找 `0xAA` 重新對齊（resync）。
- **Timer0 ISR** 以約 125 Hz 從 `fb_green[8]` / `fb_red[8]` 對點陣做多工掃描
  （8 行掃描，靠 16 bit 欄位鏈讓每行紅綠同時點亮）。
- **Timer1** 是 9600 baud 產生器；**發送是中斷驅動的環形緩衝**。
- 收到 `S_STATE`（32 byte）時存下兩塊棋盤；**VIEW** 按鈕切換顯示哪一塊
  （自己 = byte 0–15，敵方 = byte 16–31）。
- **FIRE**（開火）把游標打包成一個 byte，送出 `C_FIRE` 幀。
- 每隔約 1 秒自動重送 `C_HELLO` + `C_PLACE`（14 格的預設艦隊），直到伺服器
  回 `S_WELCOME`，所以連線時機完全不影響。

### 在 Keil uVision2 編譯
1. 新建專案 → 裝置選 **STC90C516RD+**（或任何 8052 核心；若清單沒有 STC，
   選「Atmel AT89C52」——對本程式暫存器相容）。
2. 加入 `battleship_8051.c`，設定晶振 = **11.0592 MHz**。
3. 編譯，再用 STC-ISP 燒錄 HEX。

> 提醒：伺服器的 32-byte `S_STATE` 是**兩塊棋盤**（先自己後敵方），各為
> 8 列綠色 + 8 列紅色——不是「16 綠 + 16 紅」。程式是依這個真實排列寫的。
