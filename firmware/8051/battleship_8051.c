/* ==========================================================================
 *  Battleship node  --  8051 firmware  (STC90C516RD+, Keil uVision2 / C51)
 * --------------------------------------------------------------------------
 *  Target  : STC90C516RD+  (8052-core compatible)
 *  Crystal : 24 MHz, 12T   (UART 9600 需開 SMOD=1，見第 3 節)
 *  Compiler: Keil C51  (classic C, NO C99/C11 -- traditional SFR/sbit style)
 *
 *  Responsibilities
 *  ----------------
 *   1. UART RX interrupt + state machine that parses the server framing:
 *          SYNC(0xAA) | TYPE | LENGTH | PAYLOAD[LENGTH] | CHECKSUM
 *      CHECKSUM = TYPE ^ LENGTH ^ payload[0] ^ ... ^ payload[LENGTH-1]
 *      Auto-resynchronises by hunting for 0xAA after any error.
 *
 *   2. Timer0 interrupt that multiplexes an 8x8 BI-COLOR LED matrix from a
 *      16-byte frame buffer (8 green rows + 8 red rows) via 74HC595 shifters.
 *
 *   3. Button input that packs an (X,Y) target into one byte and emits a
 *      C_FIRE frame out of the UART (interrupt-driven TX ring buffer).
 *
 *  IMPORTANT -- S_STATE layout (32 bytes from the server)
 *  ------------------------------------------------------
 *      [ 0.. 7] OWN   board GREEN rows
 *      [ 8..15] OWN   board RED   rows
 *      [16..23] ENEMY board GREEN rows
 *      [24..31] ENEMY board RED   rows
 *  i.e. it is TWO complete boards, not "16 green then 16 red".  Because this
 *  node has only ONE 8x8 matrix, we keep all 32 bytes and the VIEW button
 *  selects which board (own / enemy) is rendered.  See apply_view().
 * ========================================================================== */

#include <reg52.h>

/* ===================== 1. PROTOCOL CONSTANTS ============================== */
#define SYNC        0xAA

/* client -> server */
#define C_HELLO     0x01
#define C_PLACE     0x02
#define C_FIRE      0x03
#define C_RESET     0x0F

/* server -> client */
#define S_WELCOME   0x81
#define S_WAIT      0x82
#define S_START     0x83
#define S_STATE     0x84
#define S_TURN      0x85
#define S_RESULT    0x86
#define S_OVER      0x87
#define S_ERROR     0x8F

/* logical view selection */
#define VIEW_OWN    0
#define VIEW_ENEMY  1

/* ===================== 2. HARDWARE PIN MAP ===============================
 * 實際拓撲（與 firmware/8051/Example 一致）：共 3 顆 74HC595，分兩條鏈。
 *   Chain A (P1) = 兩顆 595 串接 = 16 bit 欄位 = 8 紅欄 + 8 綠欄
 *   Chain B (P2) = 一顆 595     = 8 bit 列選（公共端，one-hot）
 *   595 的 /OE 接 GND、/MR(/SRCLR) 接 VCC。
 * ------------------------------------------------------------------------ */
/* Chain A (P1) -- 欄位資料 (column data，兩顆串接，連送 2 byte 再鎖存)
 *   P1.2 = SER   (DS，序列資料輸入)
 *   P1.1 = SRCLK (SHCP，移位時脈，上升沿有效)
 *   P1.0 = LATCH (STCP，鎖存時脈，上升沿有效)              */
sbit SR_DATA  = P1^2;
sbit SR_CLK   = P1^1;
sbit SR_LATCH = P1^0;

/* Chain B (P2) -- 公共端 / 列選 (row select / common terminal)
 *   P2.0 = SER_B   (DS)
 *   P2.1 = SRCLK_B (SHCP)
 *   P2.2 = LATCH_B (STCP)                                  */
sbit SR_DATA_B  = P2^0;
sbit SR_CLK_B   = P2^1;
sbit SR_LATCH_B = P2^2;

/* 按鈕 -- 低電位有效（按鈕一端接腳，另一端接 GND，寫 1 啟用上拉）。
 * P2.0~P2.2 已被 Chain B 佔用，按鈕從 P2.3 起。
 * BTN_VIEW 改接 P3.2（INT0 腳，韌體不使用外部中斷）。      */
sbit BTN_UP    = P2^3;
sbit BTN_DOWN  = P2^4;
sbit BTN_LEFT  = P2^5;
sbit BTN_RIGHT = P2^6;
sbit BTN_FIRE  = P2^7;
sbit BTN_VIEW  = P3^2;

/* Matrix polarity adjusters -----------------------------------------------
 *   ROW_INVERT : set 1 if a LOW selects a row.
 *   COL_INVERT : set 1 if a LOW lights a column.
 * 本板（依 Example）：列選為高電位選通、欄位為低電位點亮。                  */
#define ROW_INVERT  0   /* 列選：高電位選通（segout 不反相） */
#define COL_INVERT  1   /* 欄位：低電位點亮（送 ~plane） */

/* ===================== 3. TIMING CONSTANTS ===============================
 * 本板：石英晶振 24 MHz、12T 模式 -> 機器週期頻率 = 24MHz/12 = 2 MHz
 *       （機器週期 = 0.5 us）。
 *
 * Timer0 = 16-bit mode，每 ~1 ms 重載一次：
 *   1 ms = 2000 機器週期 -> reload = 65536 - 2000 = 63536 = 0xF830
 *   8 rows * 1 ms = 8 ms frame -> ~125 Hz refresh（不閃爍）
 * ------------------------------------------------------------------------ */
#define T0_RELOAD_H 0xF8
#define T0_RELOAD_L 0x30

/* Timer1 (mode2) 自動重載值 + SMOD，決定 UART 鮑率：
 *   baud = (2^SMOD / 32) * (Fosc/12) / (256 - BAUD_RELOAD)
 *
 * 24MHz 在 SMOD=0 下湊不出準確的 9600（最近 10417，誤差 8.5%，會失敗）；
 * 必須開 SMOD=1（init_uart 設 PCON|=0x80）：
 *   24MHz, 12T, SMOD=1, 256-13=243=0xF3 -> baud = (2/32)*(2e6)/13 = 9615（誤差 0.16%，OK）
 *
 * 參考其他晶振（皆 SMOD=0）：
 *   11.0592MHz,12T -> 0xFD；22.1184MHz,12T -> 0xFA。換晶振時要一起改 SMOD。 */
#define BAUD_RELOAD 0xF3

/* ===================== 4. GLOBAL STATE ==================================== */

/* ---- display frame buffer (rendered by Timer0 ISR) ---- */
volatile unsigned char fb_green[8];   /* one byte per row, bit(1<<x)=col x */
volatile unsigned char fb_red[8];

/* ---- last full 32-byte S_STATE, kept so VIEW toggle can re-render ---- */
unsigned char xdata last_state[32];
unsigned char have_state = 0;

/* ---- scan / cursor state used by the display ISR ---- */
volatile unsigned char cur_row   = 0;
volatile unsigned int  frame_cnt = 0;   /* +1 per full 8-row sweep */
volatile unsigned char cursor_x  = 0;
volatile unsigned char cursor_y  = 0;
volatile bit           show_cursor = 0; /* blink a marker on the enemy board */

/* ---- game flags driven by incoming messages ---- */
volatile bit my_turn    = 0;
volatile bit got_welcome = 0;
volatile bit started    = 0;   /* 1 once S_START / S_STATE seen (game running) */
volatile bit game_over  = 0;
unsigned char view_mode = VIEW_OWN;     /* 開機預設顯示自己棋盤（一開局就看到綠色艦隊）；
                                           按 VIEW(P3.2) 切到敵方棋盤瞄準。 */
/* 勝負動畫：0=無、1=贏(綠閃)、2=輸(紅閃)。由 main loop 刷出。 */
volatile unsigned char result_state = 0;
unsigned char result_blink = 0xFF;      /* 上次閃爍相位，用來省去重複寫入 */

/* ===================== 5. UART RX STATE MACHINE =========================== */
/* Parser runs in the serial ISR; completed, checksum-verified frames are pushed
 * into a small QUEUE consumed by main(). A queue (not a 1-deep mailbox) is
 * essential: after a shot the server replies with a BURST of 3 frames back to
 * back (S_RESULT + S_STATE + S_TURN); a 1-deep mailbox would drop the 2nd/3rd
 * -> no hit/miss shown and the turn never clears.                            */
#define RX_MAX 40
static unsigned char rx_state = 0;
static unsigned char rx_type, rx_len, rx_idx, rx_chk;
static unsigned char xdata rx_buf[RX_MAX];

#define MSG_QN   4                 /* must be a power of two */
#define MSG_QMASK (MSG_QN - 1)
volatile unsigned char msg_q_type[MSG_QN];
volatile unsigned char msg_q_len[MSG_QN];
volatile unsigned char xdata msg_q_buf[MSG_QN][RX_MAX];
volatile unsigned char msg_qhead = 0;   /* ISR writes here */
volatile unsigned char msg_qtail = 0;   /* main() reads here */

/* working copy that process_message() reads (filled by main() from the queue) */
unsigned char msg_type;
unsigned char msg_len;
unsigned char xdata msg_buf[RX_MAX];

/* ===================== 6. UART TX RING BUFFER ============================ */
/* Interrupt-driven, non-blocking transmit. Size MUST be a power of two.     */
#define TX_SIZE 64
#define TX_MASK (TX_SIZE - 1)
volatile unsigned char xdata tx_buf[TX_SIZE];
volatile unsigned char tx_head = 0;     /* write index (producer = main)  */
volatile unsigned char tx_tail = 0;     /* read  index (consumer = ISR)   */
volatile bit           tx_active = 0;

/* ===================== 7. SMALL DELAY ==================================== */
/* Crude blocking delay for button debounce only (~1 ms per unit @ 24MHz/12T). */
void delay_ms(unsigned int ms)
{
    unsigned int i;
    unsigned char j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 250; j++)
            ;   /* 24MHz/12T: 1 ms = 2000 機器週期 ~= 250 * 8 cycles */
}

/* ===================== 8. 74HC595 輸出 ==================================
 * 對應 firmware/8051/Example 的 SendByte / Out595 / SendSeg：
 *   Chain A (P1)：兩顆串接，shift_byte() 連送兩個 byte 後 latch_a() 鎖存。
 *   Chain B (P2)：send_seg() 移位一個 byte 後立即鎖存。
 * ----------------------------------------------------------------------- */

/* Chain A：送一個 byte（MSB first），不鎖存。 */
void shift_byte(unsigned char dat)
{
    unsigned char i;
    for (i = 0; i < 8; i++) {
        SR_CLK = 0;
        SR_DATA = (dat & 0x80) ? 1 : 0;
        dat <<= 1;
        SR_CLK = 1;          /* 上升沿移入 */
    }
}

/* Chain A 鎖存：上升沿把移位暫存器內容送到輸出腳。 */
void latch_a(void)
{
    SR_LATCH = 1;
    SR_LATCH = 0;
}

/* Chain B（列選）：移位一個 byte 後立即鎖存。 */
void send_seg(unsigned char dat)
{
    unsigned char i;
    for (i = 0; i < 8; i++) {
        SR_CLK_B = 0;
        SR_DATA_B = (dat & 0x80) ? 1 : 0;
        dat <<= 1;
        SR_CLK_B = 1;
    }
    SR_LATCH_B = 1;
    SR_LATCH_B = 0;
}

/* 欄位「全滅」樣式（消隱用），依極性決定。 */
#if COL_INVERT
#define COL_OFF 0xFF
#else
#define COL_OFF 0x00
#endif

/* ===================== 9. 顯示多工掃描 (Timer0 ISR) =====================
 * 每次呼叫掃描「一整列」，紅綠同時點亮：
 *   1) 先消隱欄位，避免換列瞬間把新欄資料顯示在舊列上（殘影）
 *   2) Chain B 切到新列（one-hot）
 *   3) Chain A 連送兩個顏色平面後鎖存
 * 8 次 ISR = 1 完整畫面，約 125 Hz（1ms × 8）。                         */
void display_scan(void)
{
    unsigned char g, r, rowsel, bit_mask;

    g = fb_green[cur_row];
    r = fb_red[cur_row];

    /* 游標閃爍：在邏輯層疊加於綠色平面（敵方棋盤、我方回合時） */
    if (show_cursor && (cur_row == cursor_y) && ((frame_cnt & 0x10) != 0)) {
        bit_mask = (unsigned char)(1u << cursor_x);
        g ^= bit_mask;
    }

    rowsel = (unsigned char)(1u << cur_row);   /* 一熱碼列選 */

#if ROW_INVERT
    rowsel = ~rowsel;
#endif
#if COL_INVERT
    g = ~g;                 /* 低電位點亮 -> 送反相 */
    r = ~r;
#endif

    /* 1) 消隱目前欄位 */
    shift_byte(COL_OFF);
    shift_byte(COL_OFF);
    latch_a();

    /* 2) 選新列 */
    send_seg(rowsel);

    /* 3) 送兩個顏色平面。串接順序：第一個 byte 進「遠端」595。
     *    若實機紅綠顯示相反，把下面兩行對調即可。                       */
    shift_byte(r);          /* 遠端 595 -> 紅欄 */
    shift_byte(g);          /* 近端 595 -> 綠欄 */
    latch_a();

    cur_row++;
    if (cur_row >= 8) {
        cur_row = 0;
        frame_cnt++;
    }
}

void timer0_isr(void) interrupt 1
{
    TH0 = T0_RELOAD_H;          /* reload for next ~1 ms tick */
    TL0 = T0_RELOAD_L;
    display_scan();
}

/* ===================== 10. UART RX PARSER =============================== */
/* Fed one byte at a time from the serial ISR. Implements the framing state
 * machine with automatic resync on 0xAA.                                    */
void rx_feed(unsigned char b)
{
    unsigned char i;

    switch (rx_state) {
    case 0:                                 /* WAIT_SYNC */
        if (b == SYNC) rx_state = 1;        /* resync anchor */
        break;

    case 1:                                 /* TYPE */
        rx_type = b;
        rx_chk  = b;
        rx_state = 2;
        break;

    case 2:                                 /* LENGTH */
        rx_len = b;
        rx_chk ^= b;
        rx_idx = 0;
        if (rx_len == 0)            rx_state = 4;   /* no payload */
        else if (rx_len > RX_MAX)   rx_state = 0;   /* bogus -> drop, resync */
        else                        rx_state = 3;
        break;

    case 3:                                 /* PAYLOAD */
        rx_buf[rx_idx++] = b;
        rx_chk ^= b;
        if (rx_idx >= rx_len) rx_state = 4;
        break;

    case 4:                                 /* CHECKSUM */
        if (b == rx_chk) {                  /* valid frame -> push into queue */
            unsigned char nxt = (unsigned char)((msg_qhead + 1) & MSG_QMASK);
            if (nxt != msg_qtail) {         /* queue has room */
                msg_q_type[msg_qhead] = rx_type;
                msg_q_len[msg_qhead]  = rx_len;
                for (i = 0; i < rx_len; i++)
                    msg_q_buf[msg_qhead][i] = rx_buf[i];
                msg_qhead = nxt;
            }
            /* else: queue full (very rare with QN=4) -> drop oldest-safe */
        }
        rx_state = 0;                       /* always return to hunt for SYNC */
        break;

    default:
        rx_state = 0;
        break;
    }
}

/* ===================== 11. SERIAL ISR (RX + TX) ========================= */
void uart_isr(void) interrupt 4
{
    if (RI) {
        RI = 0;
        rx_feed(SBUF);
    }
    if (TI) {
        TI = 0;
        if (tx_tail != tx_head) {           /* more bytes queued? */
            SBUF   = tx_buf[tx_tail];
            tx_tail = (tx_tail + 1) & TX_MASK;
        } else {
            tx_active = 0;                  /* ring drained */
        }
    }
}

/* ===================== 12. UART TX HELPERS ============================== */
/* Enqueue one byte; kick the transmitter if idle. Blocks only if the ring
 * is momentarily full (rare -- frames are <= 36 bytes).                     */
void tx_put(unsigned char b)
{
    unsigned char nxt = (tx_head + 1) & TX_MASK;

    while (nxt == tx_tail)
        ;                                   /* wait for ISR to make room */

    tx_buf[tx_head] = b;
    tx_head = nxt;

    ES = 0;                                 /* critical section vs serial ISR */
    if (!tx_active) {
        tx_active = 1;
        TI = 0;
        SBUF   = tx_buf[tx_tail];
        tx_tail = (tx_tail + 1) & TX_MASK;
    }
    ES = 1;
}

/* Build and transmit a complete frame: SYNC TYPE LEN PAYLOAD CHK. */
void send_frame(unsigned char type, unsigned char *payload, unsigned char len)
{
    unsigned char i, chk;

    tx_put(SYNC);
    tx_put(type);
    tx_put(len);
    chk = type ^ len;
    for (i = 0; i < len; i++) {
        tx_put(payload[i]);
        chk ^= payload[i];
    }
    tx_put(chk);
}

/* ===================== 13. GAME ACTIONS ================================= */

/* Default fleet = 14 cells (4-3-3-2-2), one row-bitmask byte per row.
 * Must match the server's required ship-cell count (default 14).            */
code unsigned char DEFAULT_FLEET[8] = {
    0x0F,   /* row0: ship len 4 at x=0..3   */
    0x00,   /* row1                          */
    0x70,   /* row2: ship len 3 at x=4..6   */
    0x00,   /* row3                          */
    0x07,   /* row4: ship len 3 at x=0..2   */
    0x00,   /* row5                          */
    0xC0,   /* row6: ship len 2 at x=6..7   */
    0x18    /* row7: ship len 2 at x=3..4   */
};

void send_hello(void)
{
    send_frame(C_HELLO, 0, 0);
}

void send_placement(void)
{
    unsigned char tmp[8];
    unsigned char i;
    for (i = 0; i < 8; i++)
        tmp[i] = DEFAULT_FLEET[i];          /* copy from code space to a RAM buf */
    send_frame(C_PLACE, tmp, 8);
}

/* Pack target (x,y) into one byte (X high nibble, Y low) and fire. */
void send_fire(unsigned char x, unsigned char y)
{
    unsigned char coord = (unsigned char)((x << 4) | (y & 0x0F));
    send_frame(C_FIRE, &coord, 1);
}

/* ===================== 14. STATE -> FRAME BUFFER ======================== */
/* Copy the selected board (own or enemy) out of last_state[] into the live
 * frame buffer. Disable the display ISR briefly so the 16 bytes update as a
 * coherent set (no tearing).                                                */
void apply_view(void)
{
    unsigned char i, base;

    if (!have_state) return;

    base = (view_mode == VIEW_ENEMY) ? 16 : 0;   /* enemy board starts at 16 */

    ET0 = 0;                                     /* freeze the scanner */
    for (i = 0; i < 8; i++) {
        fb_green[i] = last_state[base + i];
        fb_red[i]   = last_state[base + 8 + i];
    }
    ET0 = 1;

    /* Show the firing cursor only on the enemy board while it is our turn. */
    show_cursor = (view_mode == VIEW_ENEMY) && my_turn && !game_over;
}

/* 勝負動畫：贏 -> 全綠閃爍；輸 -> 全紅閃爍。由 main loop 在 result_state 時呼叫。
 * 只在閃爍相位改變時更新（每 ~0.25s 兩次），避免一直占用 CPU 影響掃描。 */
void show_result(void)
{
    unsigned char i, on;

    on = (frame_cnt & 0x20) ? 1 : 0;     /* ~0.25s 切換一次 */
    if (on == result_blink) return;
    result_blink = on;

    ET0 = 0;
    for (i = 0; i < 8; i++) {
        if (result_state == 1) {         /* 贏：綠 */
            fb_green[i] = on ? 0xFF : 0x00;
            fb_red[i]   = 0x00;
        } else {                         /* 輸：紅 */
            fb_red[i]   = on ? 0xFF : 0x00;
            fb_green[i] = 0x00;
        }
    }
    ET0 = 1;
}

/* ===================== 15. MESSAGE HANDLER (main loop) ================== */
void process_message(void)
{
    unsigned char i;

    switch (msg_type) {

    case S_WELCOME:
        got_welcome = 1;
        /* 連線此刻一定已建立，立刻補送一次佈艦，避免「開機時送的 PLACE
         * 在 TCP 還沒連上前就被 ESP32 丟掉」而導致伺服器永遠收不到佈艦。 */
        send_placement();
        /* msg_buf[0] = our player id (0/1) -- not needed for rendering */
        break;

    case S_WAIT:
        /* opponent / placement pending -- nothing to draw */
        break;

    case S_START:
        started = 1;           /* 遊戲已開始 -> 停止重送握手 */
        game_over = 0;
        result_state = 0;      /* 清掉上一局的勝負動畫 */
        my_turn = (msg_len >= 1) ? (msg_buf[0] ? 1 : 0) : 0;
        break;

    case S_STATE:
        if (msg_len == 32) {
            started = 1;       /* 安全網：收到棋盤也代表已開局 */
            for (i = 0; i < 32; i++)
                last_state[i] = msg_buf[i];
            have_state = 1;
            apply_view();
        }
        break;

    case S_TURN:
        my_turn = (msg_len >= 1) ? (msg_buf[0] ? 1 : 0) : 0;
        /* 換到我方回合：對手剛打完，自動切回「自己棋盤」讓玩家先看有沒有被
         * 打中；要瞄準時再按 VIEW 切到敵方棋盤。 */
        if (my_turn && !game_over) {
            view_mode = VIEW_OWN;
            apply_view();
        }
        show_cursor = (view_mode == VIEW_ENEMY) && my_turn && !game_over;
        break;

    case S_RESULT:
        /* msg_buf[0] = coord we fired at, msg_buf[1] = 1 hit / 0 miss.
         * (Hook your buzzer / status LED here if desired.)                  */
        break;

    case S_OVER:
        game_over = 1;
        my_turn = 0;
        show_cursor = 0;
        /* msg_buf[0] = 1 贏 / 0 輸 -> 觸發勝負動畫（main loop 刷出）。 */
        result_state = (msg_len >= 1 && msg_buf[0]) ? 1 : 2;
        result_blink = 0xFF;   /* 強制下一次 show_result 立即更新 */
        break;

    case S_ERROR:
        /* msg_buf[0] = error code。刻意忽略：開局後若有一個遲到的重複 PLACE
         * 會收到 E_STATE，這屬正常，不可藉此重置握手（否則會進入錯誤迴圈）。
         * 需要重新配對時直接按 8051 的 RESET 鍵即可。                         */
        break;

    default:
        break;
    }
}

/* ===================== 16. 按鈕輸入 ===================================== */
/* 簡易輪詢 + 去彈跳 + 邊緣偵測。低電位有效。
 * P2.0~P2.2 已被 Chain B 佔用；按鈕分布在 P2.3~P2.7 與 P3.2。             */
static unsigned char btn_prev_p2 = 0xFF;
static unsigned char btn_prev_p3 = 0xFF;
static unsigned int  btn_lock = 0;      /* lockout timestamp (frame_cnt) */

/* 非阻塞去彈跳：用 frame_cnt 做「鎖定」式去彈跳，絕不 delay_ms 卡住主迴圈，
 * 否則開火後的 3 連幀(S_RESULT/S_STATE/S_TURN)可能在阻塞期間被 ISR 丟掉。 */
void poll_buttons(void)
{
    unsigned char now_p2, now_p3, changed_p2, changed_p3;

    now_p2 = P2 | 0x07;    /* 屏蔽 P2.0~P2.2（Chain B 腳位） */
    now_p3 = P3 | 0xFB;    /* 僅讀 P3.2（BTN_VIEW），其餘保持 1 */

    if (now_p2 == btn_prev_p2 && now_p3 == btn_prev_p3) return;  /* 無變化 */
    if ((unsigned int)(frame_cnt - btn_lock) < 4) return;       /* ~32ms 鎖定 */

    changed_p2 = (unsigned char)(btn_prev_p2 & ~now_p2);
    changed_p3 = (unsigned char)(btn_prev_p3 & ~now_p3);
    btn_prev_p2 = now_p2;
    btn_prev_p3 = now_p3;
    btn_lock = frame_cnt;

    if (changed_p2 & 0x08) {               /* BTN_UP    P2.3 */
        cursor_y = (cursor_y == 0) ? 7 : (cursor_y - 1);
    }
    if (changed_p2 & 0x10) {               /* BTN_DOWN  P2.4 */
        cursor_y = (cursor_y + 1) & 7;
    }
    if (changed_p2 & 0x20) {               /* BTN_LEFT  P2.5 */
        cursor_x = (cursor_x == 0) ? 7 : (cursor_x - 1);
    }
    if (changed_p2 & 0x40) {               /* BTN_RIGHT P2.6 */
        cursor_x = (cursor_x + 1) & 7;
    }
    if (changed_p2 & 0x80) {               /* BTN_FIRE  P2.7 */
        if (my_turn && !game_over)
            send_fire(cursor_x, cursor_y);
    }
    if (changed_p3 & 0x04) {               /* BTN_VIEW  P3.2 */
        if (!result_state) {               /* 勝負動畫進行中不切換 */
            view_mode = (view_mode == VIEW_ENEMY) ? VIEW_OWN : VIEW_ENEMY;
            apply_view();
        }
    }
}

/* ===================== 17. PERIPHERAL INIT ============================== */
void init_io(void)
{
    /* Chain A（欄位資料）初始狀態：LATCH 閒置為低（鎖存在上升沿） */
    SR_DATA  = 0;
    SR_CLK   = 0;
    SR_LATCH = 0;

    /* Chain B（公共端 / 列選）初始狀態：LATCH 閒置為低 */
    SR_DATA_B  = 0;
    SR_CLK_B   = 0;
    SR_LATCH_B = 0;

    /* 按鈕輸入：P2.3~P2.7 及 P3.2 寫 1 啟用準雙向上拉 */
    P2 |= 0xF8;
    P3 |= 0x04;

    /* 清除顯示緩衝 */
    {
        unsigned char i;
        for (i = 0; i < 8; i++) {
            fb_green[i] = 0;
            fb_red[i]   = 0;
        }
    }
}

void init_timer0(void)
{
    /* Timer0 = mode 1 (16-bit). Combined later with Timer1 in TMOD. */
    TH0 = T0_RELOAD_H;
    TL0 = T0_RELOAD_L;
    ET0 = 1;                   /* enable Timer0 interrupt */
    TR0 = 1;                   /* run Timer0 */
}

void init_uart(void)
{
    /* Timer1 = mode 2 (8-bit auto-reload) as the baud-rate generator. */
    TH1 = BAUD_RELOAD;
    TL1 = BAUD_RELOAD;
    PCON |= 0x80;             /* SMOD = 1（雙速）：24MHz 才湊得出 9600，見第 3 節 */
    TR1 = 1;                   /* run Timer1 (baud clock) */

    SCON = 0x50;              /* mode 1 (8-bit UART), REN=1 (rx enabled)   */
    ES   = 1;                  /* enable serial interrupt */
}

/* ===================== 18. MAIN ========================================= */
void main(void)
{
    unsigned int last_hello = 0;

    init_io();

    /* TMOD: Timer1 mode2 (baud) = 0x20  |  Timer0 mode1 (refresh) = 0x01 */
    TMOD = 0x21;

    init_timer0();
    init_uart();

    EA = 1;                    /* global interrupt enable */

    /* Kick off the handshake. The ESP32 only forwards bytes once its TCP
     * socket is up, so we (re)send HELLO+PLACE periodically until the game
     * actually STARTS.  關鍵：必須撐到 S_START，而不是只到 S_WELCOME——因為
     * S_WELCOME 在 TCP 一連上就到，可能早於任何一次「連線後」的 PLACE 被送達，
     * 若此時就停送，伺服器將永遠收不到本機佈艦，雙方卡在等待。               */
    send_hello();
    send_placement();
    last_hello = frame_cnt;

    for (;;) {

        /* 1) drain ALL queued inbound frames (a post-shot burst is 3 frames:
         *    S_RESULT + S_STATE + S_TURN -- process them all this pass). */
        while (msg_qtail != msg_qhead) {
            unsigned char s = msg_qtail;
            unsigned char j;
            msg_type = msg_q_type[s];
            msg_len  = msg_q_len[s];
            for (j = 0; j < msg_len; j++)
                msg_buf[j] = msg_q_buf[s][j];
            msg_qtail = (unsigned char)((s + 1) & MSG_QMASK);
            process_message();
        }

        /* 2) 重送 HELLO+PLACE 直到開局（~1 秒一次：125 frames/sec） */
        if (!started) {
            if ((unsigned int)(frame_cnt - last_hello) > 125) {
                send_hello();
                send_placement();
                last_hello = frame_cnt;
            }
        }

        /* 3) 勝負動畫（贏=綠閃 / 輸=紅閃） */
        if (result_state) {
            show_result();
        }

        /* 4) read the keypad and act */
        poll_buttons();
    }
}
