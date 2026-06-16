# ==========================================================================
#  Battleship node  --  ESP32 serial-to-Wi-Fi bridge  (MicroPython)
# --------------------------------------------------------------------------
#  角色：在 Python 伺服器的 raw TCP socket 與 8051 的 UART 之間，做
#        「透明、雙向、逐 byte」搬運。本程式完全不解析遊戲協議
#        （SYNC/TYPE/LEN/PAYLOAD/CHECKSUM 全由 8051 與伺服器負責）。
#
#  存放：把本檔以檔名 main.py 放進 ESP32 檔系統，開機後自動執行。
#        （需先燒錄 MicroPython 韌體：esptool 抹除 + 寫入 esp32 .bin）
#
#  接線（UART2 <-> 8051，3.3V 邏輯，務必對 8051 的 5V 做電平轉換）：
#      ESP32 GPIO17 (TX) ----[電平轉換]----> 8051 RXD (P3.0)
#      ESP32 GPIO16 (RX) <---[電平轉換]----- 8051 TXD (P3.1)
#      GND <-> GND（共地）
#
#  Baud 必須與 8051 一致：9600 8N1（8051 用 11.0592MHz 晶振）。
# ==========================================================================

import sys
import time
import network
import socket
import select

try:
    import errno
except ImportError:          # 某些移植版叫 uerrno
    import uerrno as errno

from machine import UART, Pin

# ----------------------------- 使用者設定 --------------------------------
WIFI_SSID = "TP-LINK_8364"
WIFI_PASS = "0965213356"
SERVER_HOST = "192.168.233.104"     # Python 伺服器 IP
SERVER_PORT = 9000                # Python 伺服器埠

UART_ID = 2                       # 用 UART2，把 USB(REPL) 的 UART0 留給除錯
UART_BAUD = 9600                  # 對齊 8051：24MHz/12T + SMOD=1 + TH1=0xF3 = 9615 ≈ 9600
UART_TX = 17                      # ESP32 TX -> 8051 RXD
UART_RX = 16                      # ESP32 RX <- 8051 TXD

LED_PIN = 2                       # 板載 LED：連上伺服器時恆亮（沒有就設 None）
# -------------------------------------------------------------------------

BUF = 512            # 整批搬運緩衝；遠大於最大幀（4+32=36）
POLL_MS = 50         # poll 逾時：兼顧延遲與 CPU 佔用
RECONNECT_S = 1      # 重連間隔（秒）
DEBUG = True         # True：每 2 秒在 REPL 印出雙向位元組數（除錯完可設 False）

led = Pin(LED_PIN, Pin.OUT) if LED_PIN is not None else None


def set_led(on):
    if led is not None:
        led.value(1 if on else 0)


# --------------------------------------------------------------------------
# Wi-Fi：確保連線（首次會等待關聯，之後靠自動重連）
# --------------------------------------------------------------------------
def ensure_wifi(wlan):
    if wlan.isconnected():
        return True

    print("[wifi] connecting to", WIFI_SSID)
    wlan.active(True)
    wlan.connect(WIFI_SSID, WIFI_PASS)

    t = 0
    while not wlan.isconnected() and t < 100:   # 最多約 10 秒
        time.sleep_ms(100)
        t += 1

    if wlan.isconnected():
        print("[wifi] connected, ifconfig =", wlan.ifconfig())
        return True
    print("[wifi] connect failed, will retry")
    return False


# --------------------------------------------------------------------------
# TCP：建立到伺服器的連線，並關閉 Nagle（小幀要立刻送出）
# --------------------------------------------------------------------------
def connect_server():
    try:
        addr = socket.getaddrinfo(SERVER_HOST, SERVER_PORT)[0][-1]
    except OSError as e:
        print("[tcp] getaddrinfo failed:", e)
        return None

    print("[tcp] connecting to", SERVER_HOST, SERVER_PORT)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect(addr)
    except OSError as e:
        print("[tcp] connect failed:", e)
        sock.close()
        return None

    # 關閉 Nagle：我們的幀很小（4~36B），必須立即送出不可合併。
    # 不同移植版的常數名稱可能缺失，逐層退回到 lwIP 數值。
    try:
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    except (AttributeError, OSError):
        try:
            sock.setsockopt(6, 0x01, 1)   # IPPROTO_TCP=6, TCP_NODELAY=1
        except OSError:
            pass

    print("[tcp] connected")
    return sock


# --------------------------------------------------------------------------
# 可靠送出整段資料（非阻塞 socket 上處理 EAGAIN / 部分寫入）
# --------------------------------------------------------------------------
def send_all(sock, data):
    mv = memoryview(data)
    while mv:
        try:
            n = sock.write(mv)
        except OSError as e:
            if e.args[0] == errno.EAGAIN:
                time.sleep_ms(1)
                continue
            raise                 # 其它錯誤 -> 視為斷線，往上拋
        if n is None:             # 非阻塞下暫時送不出
            time.sleep_ms(1)
            continue
        mv = mv[n:]


# --------------------------------------------------------------------------
# 透明管道主迴圈：兩個方向都搬，直到斷線拋出 OSError
# --------------------------------------------------------------------------
def run_pipe(wlan, sock, uart):
    sock.setblocking(False)       # 讀取改非阻塞，靠 poll 與回傳值判斷

    poller = select.poll()
    poller.register(sock, select.POLLIN)
    try:
        poller.register(uart, select.POLLIN)   # 多數 ESP32 版支援 UART 進 poll
    except (OSError, ValueError):
        pass                      # 不支援也沒關係，下面每圈都會檢查 uart.any()

    set_led(True)
    n_up = 0          # 累計 8051 -> 伺服器 的位元組數（除錯用）
    n_down = 0        # 累計 伺服器 -> 8051 的位元組數
    last_report = time.ticks_ms()
    try:
        while True:
            if not wlan.isconnected():
                raise OSError("wifi lost")

            poller.poll(POLL_MS)  # 任一端有資料或逾時就醒來

            # ---- TCP -> UART（伺服器 -> 8051）----
            data = sock.read(BUF)            # 非阻塞：無資料回 None，斷線回 b''
            if data == b"":
                raise OSError("server closed")
            if data:
                uart.write(data)
                n_down += len(data)

            # ---- UART -> TCP（8051 -> 伺服器）----
            n = uart.any()
            if n:
                d = uart.read(n)             # 只讀「已到」的位元組，不等待
                if d:
                    send_all(sock, d)
                    n_up += len(d)
                    if DEBUG:                # 印出 8051 送來的原始 hex
                        print("[rx]", " ".join("%02X" % b for b in d))

            # ---- 除錯：每 2 秒回報雙向位元組數（確認哪一段在動）----
            if DEBUG:
                now = time.ticks_ms()
                if time.ticks_diff(now, last_report) > 2000:
                    print("[dbg] 8051->server:", n_up, "B   server->8051:", n_down, "B")
                    last_report = now
    finally:
        set_led(False)
        try:
            poller.unregister(sock)
        except Exception:
            pass
        try:
            poller.unregister(uart)
        except Exception:
            pass


# --------------------------------------------------------------------------
def main():
    # UART2 接 8051；加大收發緩衝以吸收突發。
    uart = UART(UART_ID, baudrate=UART_BAUD, tx=UART_TX, rx=UART_RX,
                bits=8, parity=None, stop=1, txbuf=1024, rxbuf=1024)

    wlan = network.WLAN(network.STA_IF)
    set_led(False)
    print("[boot] Battleship ESP32 (MicroPython) bridge starting")

    while True:
        try:
            if not ensure_wifi(wlan):
                time.sleep(2)
                continue

            sock = connect_server()
            if sock is None:
                time.sleep(RECONNECT_S)
                continue

            try:
                run_pipe(wlan, sock, uart)
            finally:
                try:
                    sock.close()
                except Exception:
                    pass

        except OSError as e:
            print("[link] reset:", e)        # 斷線/錯誤 -> 重連
        except Exception as e:               # 防呆：任何例外都別讓 bridge 死掉
            print("[fatal] unexpected:", e)
            sys.print_exception(e)

        time.sleep(RECONNECT_S)


main()
