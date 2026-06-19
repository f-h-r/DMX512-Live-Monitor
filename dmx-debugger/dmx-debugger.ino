/*
  DMX512 Live Monitor with TFT Display – Arduino Mega 2560
  =========================================================
  Hardware:
    DMX input    : Serial1 / RX1 (Pin 19) via RS485 transceiver (RE/DE tied to GND)
    TFT display  : 2.4" shield (ILI9341, 240x320, 8-bit parallel + resistive touch)
    USB serial   : Serial0 (optional serial monitor, see flag below)

  ── Required libraries – install via Library Manager ─────────────────────────

    1. DMXSerial             Matthias Hertel
       Search in Library Manager: "DMXSerial"
       After installation: uncomment "#define DMX_USE_PORT1" in <DMXSerial_avr.h>
       to enable Serial1 (hardware UART 1, Pin 19).

    2. MCUFRIEND_kbv         David Prentice
       Search: "MCUFRIEND_kbv"
       8-bit parallel TFT driver; auto-detects controller via readID().
       Supports ILI9341, ILI9325, HX8347, and others.

    3. Adafruit GFX Library  Adafruit Industries
       Search: "Adafruit GFX Library"
       Required dependency of MCUFRIEND_kbv.

    4. Adafruit BusIO        Adafruit Industries
       Search: "Adafruit BusIO"
       Required dependency of Adafruit GFX Library.

    5. TouchScreen           Adafruit Industries
       Search: "TouchScreen"
       Note: TOUCH_XP/YP/XM/YM pins and calibration values (TS_MINX etc.)
       may need adjustment for your specific shield.
       Calibration sketch: "TouchScreen_Calibr_native" in MCUFRIEND_kbv examples.

  ── Serial debug output ───────────────────────────────────────────────────────
    Default: disabled (saves SRAM, reduces runtime overhead).
    Enable: uncomment "#define SERIAL_OUTPUT_ENABLE" below.

  __ Build and Upload
    cd "C:\Program Files (x86)\Arduino"; .\arduino_debug.exe \
    --board arduino:avr:mega:cpu=atmega2560 --port COM3 \
    --upload "C:\Users\rp-remote\Documents\Arduino\dmx-debugger\dmx-debugger.ino"    

  ── Pages / touch navigation ─────────────────────────────────────────────────
    Page 1  All 512 channels as bar chart (16 cols x 32 rows)
              Value 0: border only visible; value 255: fully filled
              Fill colour follows grayscale scheme (dark to light)
    Page 2  Channels   1–256 as decimal values (10 cols x 26 rows), right-aligned
    Page 3  Channels 257–512 as decimal values (10 cols x 26 rows), right-aligned

    Touch left  half → previous page (wraps from page 1 to page 3)
    Touch right half → next page     (wraps from page 3 to page 1)
    Note: touch is suppressed for the first 1000 ms after boot (anti-ghost).

  ── Header (dynamic) ─────────────────────────────────────────────────────────
    Line 1 : "DMX512 Monitor"  left  |  "X/3" right-aligned  (X = current page)
    Line 2 : coloured status box left (green = OK, red = FAIL) | page name right

  ── Pages 2/3 layout ─────────────────────────────────────────────────────────
    x   0 –  26 : row header (first channel number of the row, right-aligned)  27 px
    x      27   : vertical separator                                             1 px
    x  28 – 237 : 10 data columns (20 px content + 1 px right separator each)
    Total: 238 px <= 240 px  ✓
    y  20 –  29 : column headers (offsets 0–9)
    y  30 – 289 : 26 data rows (9 px content + 1 px bottom separator) = 260 px  ✓

  ── Colour coding ─────────────────────────────────────────────────────────────
    Dark grey  : value = 0          (~15 % L)
    Mid grey   : value   1 –  99    (~38 % L)
    Light grey : value 100 – 199    (~62 % L)
    White-grey : value 200 – 255    (~85 % L)
    Yellow     : changed value (stays highlighted for HIGHLIGHT_TICKS frames)

  ── Performance optimisations ─────────────────────────────────────────────────
    1. pushColors() instead of fillRect() + print() per cell:
       After setAddrWindow(), pixel colours are written as a raw burst via
       pushColors() — eliminates per-cell CS/CD toggle overhead and bypasses
       the Adafruit GFX text renderer entirely.
       Page 1 cell: 14 * 8 = 112 pixels, 2 bytes/pixel → 224 bytes per pushColors.
    2. fillScreen() is used for full clears because MCUFRIEND_kbv performs an
       internal pixel burst over all 76800 pixels, which is the fastest way
       to write a single colour value to the entire display.
    3. Ghost-touch suppression: lastTouchMs is initialised so that the first
       TOUCH_BOOT_MS after reset produce no touch reaction.
    4. Touch pins: all four pins (XP, YP, XM, YM) are reset to OUTPUT after
       getPoint() so they do not interfere with the 8-bit TFT bus.
*/

// ── Serial debug output: uncomment the line below to enable ─────────────────
// #define SERIAL_OUTPUT_ENABLE

#include <DMXSerial.h>
#include <Adafruit_GFX.h>
#include <MCUFRIEND_kbv.h>
#include <TouchScreen.h>

// ── Touch pins (adjust to match your shield schematic if necessary) ──────────
#define TOUCH_XP          8     // digital
#define TOUCH_YP         A3     // must be analog
#define TOUCH_XM         A2     // must be analog
#define TOUCH_YM          9     // digital
#define TOUCH_RESIST    300     // plate resistance in ohms (see shield datasheet)

// ── Touch calibration (determine values with TouchScreen_Calibr_native) ────────
#define TS_MINX          120
#define TS_MAXX          900
#define TS_MINY           70
#define TS_MAXY          920
#define MINPRESSURE      200
#define MAXPRESSURE     1000
#define TOUCH_COOLDOWN_MS  400U  // minimum interval [ms] between page changes
#define TOUCH_BOOT_MS     1000U  // ghost-touch suppression window after reset [ms]

// ── Timing and system constants ──────────────────────────────────────────────
static const uint32_t DEBUG_BAUD      = 115200UL;
static const uint16_t DMX_CHANNELS    = 512;
static const uint32_t REFRESH_MS      = 250UL;
static const uint8_t  HIGHLIGHT_TICKS = 3;
static const uint32_t DMX_TIMEOUT_MS  = 500UL;

// ── Shared layout ────────────────────────────────────────────────────────────
static const uint8_t  HEADER_H  = 20;   // header height [px]
static const uint8_t  COLHDR_H  = 10;   // column header height [px]
static const uint8_t  GRID_Y    = HEADER_H + COLHDR_H;  // = 30

// ── Page 1 layout (16 cols x 32 rows = 512 channels) ────────────────────────
//   16 px row header + 16 * 14 px = 240 px  |  30 px header + 32 * 9 px <= 320 px
static const uint8_t  P1_COLS      = 16;
static const uint8_t  P1_ROWHDR_W  = 16;   // row header width [px]
static const uint8_t  P1_CELL_W    = 14;   // cell slot width [px]
static const uint8_t  P1_CELL_H    =  9;   // cell slot height [px]
static const uint8_t  P1_PX_W      = 13;   // usable pixel width per cell
static const uint8_t  P1_PX_H      =  8;   // usable pixel height per cell

// ── Pages 2/3 layout (decimal, 10 cols x 26 rows, right-aligned) ────────────
static const uint8_t  P23_COLS     = 10;
static const uint8_t  P23_CELL_W   = 20;
static const uint8_t  P23_CELL_H   = 10;
static const uint8_t  P23_ROWHDR_W = 27;
static const uint8_t  P23_COL_X0   = 28;
static const uint8_t  P23_SLOT     = 21;
// Usable pixel area of a data cell
static const uint8_t  P23_PX_W     = 20;
static const uint8_t  P23_PX_H     =  9;

// ── RGB565 colours ───────────────────────────────────────────────────────────
static const uint16_t C_BG      = 0x0000;  // black
static const uint16_t C_HDR_BG  = 0x000F;  // header background: dark blue
static const uint16_t C_HDR_FG  = 0xFFFF;  // header text:       white
static const uint16_t C_COL_BG  = 0x0841;  // column header background
static const uint16_t C_COL_FG  = 0x7BEF;  // column header text: light grey
static const uint16_t C_GRID    = 0x4208;  // grid lines:         dark grey
static const uint16_t C_ZERO    = 0x2945;  // value = 0:          dark grey   (~15 % L)
static const uint16_t C_LOW     = 0x630C;  // value   1 –  99:    mid grey    (~38 % L)
static const uint16_t C_MID     = 0x9CF3;  // value 100 – 199:    light grey  (~62 % L)
static const uint16_t C_HIGH    = 0xD6BA;  // value 200 – 255:    white-grey  (~85 % L)
static const uint16_t C_CHG_BG  = 0x2104;  // highlight background: dark gold
static const uint16_t C_CHG_FG  = 0xFFE0;  // highlight text:       yellow
static const uint16_t C_OK      = 0x0300;  // dark green (status: OK)
static const uint16_t C_FAIL    = 0xF800;  // red        (status: FAIL)

// ── Global objects ───────────────────────────────────────────────────────────
MCUFRIEND_kbv tft;
TouchScreen   ts(TOUCH_XP, TOUCH_YP, TOUCH_XM, TOUCH_YM, TOUCH_RESIST);

// ── Buffers (3 x 512 = 1536 bytes SRAM) ─────────────────────────────────────
uint8_t snapshot[DMX_CHANNELS];
uint8_t prevSnapshot[DMX_CHANNELS];
uint8_t flashTimer[DMX_CHANNELS];

// ── State variables ──────────────────────────────────────────────────────────
uint8_t  currentPage = 1;
bool     dmxOk       = false;

// ── pushColors pixel buffer: max cell size P23 = 20*9 = 180 pixels = 360 bytes
// Static to avoid stack risk for nested calls
static uint16_t pxBuf[P23_PX_W * P23_PX_H];  // 360 bytes SRAM

// ── Pages 2/3 helper functions ───────────────────────────────────────────────
static inline uint16_t p23StartIdx(uint8_t page) { return (page == 2) ? 0   : 256; }
static inline uint16_t p23ChCount(uint8_t page)  { return (page == 2) ? 256 : 256; }
static inline uint8_t  p23RowCount(uint8_t page) {
  return (uint8_t)((p23ChCount(page) + P23_COLS - 1) / P23_COLS);
}

// ── Foreground colour for a given channel value ─────────────────────────────
static inline uint16_t valueColor(uint8_t v) {
  if (v ==  0) return C_ZERO;
  if (v < 100) return C_LOW;
  if (v < 200) return C_MID;
  return C_HIGH;
}

// ── Fill a cell area with a solid colour and write it via pushColors ────────
// Bypasses the Adafruit GFX text renderer entirely:
// setAddrWindow -> single pushColors burst instead of N x fillRect + print
static void fillCellFast(uint16_t x, uint16_t y,
                          uint8_t  pw, uint8_t ph,
                          uint16_t color) {
  const uint16_t n = (uint16_t)pw * ph;
  for (uint16_t i = 0; i < n; i++) pxBuf[i] = color;
  tft.setAddrWindow(x, y, x + pw - 1, y + ph - 1);
  tft.pushColors(pxBuf, n, true);
}

// ── 4x5 pixel font data (digits 0-9, hex A-F) stored in PROGMEM ─────────────
// Each glyph: 5 rows of 4 bits (MSB = leftmost pixel); 0 = background, 1 = foreground
// Format: {row0, row1, row2, row3, row4}
// Width 4 px, height 5 px — fits well in 14x8 px cells (P1) and 20x9 px (P23)
static const uint8_t FONT4x5[][5] PROGMEM = {
  {0b0110, 0b1001, 0b1001, 0b1001, 0b0110},  // 0
  {0b0010, 0b0110, 0b0010, 0b0010, 0b0111},  // 1
  {0b0110, 0b1001, 0b0010, 0b0100, 0b1111},  // 2
  {0b1110, 0b0001, 0b0110, 0b0001, 0b1110},  // 3
  {0b1001, 0b1001, 0b1111, 0b0001, 0b0001},  // 4
  {0b1111, 0b1000, 0b1110, 0b0001, 0b1110},  // 5
  {0b0110, 0b1000, 0b1110, 0b1001, 0b0110},  // 6
  {0b1111, 0b0001, 0b0010, 0b0100, 0b0100},  // 7
  {0b0110, 0b1001, 0b0110, 0b1001, 0b0110},  // 8
  {0b0110, 0b1001, 0b0111, 0b0001, 0b0110},  // 9
  {0b0110, 0b1001, 0b1111, 0b1001, 0b1001},  // A
  {0b1110, 0b1001, 0b1110, 0b1001, 0b1110},  // B
  {0b0110, 0b1001, 0b1000, 0b1001, 0b0110},  // C
  {0b1110, 0b1001, 0b1001, 0b1001, 0b1110},  // D
  {0b1111, 0b1000, 0b1110, 0b1000, 0b1111},  // E
  {0b1111, 0b1000, 0b1110, 0b1000, 0b1000},  // F
};

// ── Blit a single glyph (digit / hex letter) into pxBuf ─────────────────────
// cx, cy: top-left pixel offset within the buffer (0-based; buffer row width = stride)
// charIdx: 0-15 for '0'-'F'
static void blitChar(uint8_t charIdx, uint8_t cx, uint8_t cy,
                     uint8_t stride, uint16_t fg, uint16_t bg) {
  for (uint8_t row = 0; row < 5; row++) {
    uint8_t bits = pgm_read_byte(&FONT4x5[charIdx][row]);
    for (uint8_t col = 0; col < 4; col++) {
      pxBuf[(cy + row) * stride + cx + col] =
          (bits & (0x08 >> col)) ? fg : bg;
    }
  }
}

// Draw a single 4x5 bitmap character directly to the screen at (sx, sy).
// Uses a 20-pixel stack buffer (40 bytes) — safe on AVR.
static void drawBitmapChar(uint8_t charIdx, uint16_t sx, uint16_t sy,
                           uint16_t fg, uint16_t bg) {
  uint16_t buf[20];
  for (uint8_t row = 0; row < 5; row++) {
    uint8_t bits = pgm_read_byte(&FONT4x5[charIdx][row]);
    for (uint8_t col = 0; col < 4; col++) {
      buf[row * 4 + col] = (bits & (0x08 >> col)) ? fg : bg;
    }
  }
  tft.setAddrWindow(sx, sy, sx + 3, sy + 4);
  tft.pushColors(buf, 20, true);
}

// Draw a decimal number (1-3 digits) right-aligned, last char ending at rx.
// Vertical baseline: sy. Digit gap: 1 px. Char width: 4 px.
static void drawBitmapDecimal(uint16_t val, uint16_t rx, uint16_t sy,
                              uint16_t fg, uint16_t bg) {
  if (val >= 100) {
    drawBitmapChar(val / 100,       rx - 10, sy, fg, bg);
    drawBitmapChar((val / 10) % 10, rx -  5, sy, fg, bg);
    drawBitmapChar(val % 10,        rx,      sy, fg, bg);
  } else if (val >= 10) {
    drawBitmapChar(val / 10,  rx - 5, sy, fg, bg);
    drawBitmapChar(val % 10,  rx,     sy, fg, bg);
  } else {
    drawBitmapChar(val, rx, sy, fg, bg);
  }
}

// ── Page 1 cell: bar chart (13x8 px via pushColors) ─────────────────────────
// 1 px border (C_GRID, or C_CHG_FG when highlighted).
// Inner area 11x6 px: filled from the left with valueColor(val) for barW pixels,
// remainder is background. Fill width = round(val * 11 / 255).
static void drawP1Cell(uint16_t idx, uint8_t val, bool highlight) {
  const uint8_t  col = (uint8_t)(idx % P1_COLS);
  const uint8_t  row = (uint8_t)(idx / P1_COLS);
  const uint16_t x   = (uint16_t)P1_ROWHDR_W + (uint16_t)col * P1_CELL_W;
  const uint16_t y   = (uint16_t)GRID_Y + (uint16_t)row * P1_CELL_H;

  const uint16_t barCol    = highlight ? C_CHG_FG : valueColor(val);
  const uint16_t bgCol     = highlight ? C_CHG_BG : C_BG;
  const uint16_t borderCol = highlight ? C_CHG_FG : C_GRID;

  // Inner width = P1_PX_W - 2 = 12; bar width proportional to value
  const uint8_t innerW = P1_PX_W - 2;  // 12
  const uint8_t barW   = (val == 0) ? 0 : (uint8_t)(((uint16_t)val * innerW + 127) / 255);

  // Build pixel buffer row by row
  for (uint8_t py = 0; py < P1_PX_H; py++) {
    for (uint8_t px2 = 0; px2 < P1_PX_W; px2++) {
      uint16_t c;
      if (py == 0 || py == P1_PX_H - 1 || px2 == 0 || px2 == P1_PX_W - 1) {
        c = borderCol;
      } else if ((px2 - 1) < barW) {
        c = barCol;
      } else {
        c = bgCol;
      }
      pxBuf[py * P1_PX_W + px2] = c;
    }
  }

  tft.setAddrWindow(x, y, x + P1_PX_W - 1, y + P1_PX_H - 1);
  tft.pushColors(pxBuf, (uint16_t)P1_PX_W * P1_PX_H, true);
}

static void drawAllP1Cells() {
  for (uint16_t i = 0; i < DMX_CHANNELS; i++) {
    drawP1Cell(i, snapshot[i], false);
  }
}

static void updateP1Cells() {
  for (uint16_t i = 0; i < DMX_CHANNELS; i++) {
    const bool isNew = (snapshot[i] != prevSnapshot[i]);
    if (isNew) {
      flashTimer[i] = HIGHLIGHT_TICKS;
      drawP1Cell(i, snapshot[i], true);
    } else if (flashTimer[i] > 0) {
      flashTimer[i]--;
      if (flashTimer[i] == 0) drawP1Cell(i, snapshot[i], false);
    }
  }
}

// ── Draw the complete header ─────────────────────────────────────────────────
// Line 1: "DMX512 Monitor" left  |  "X/3" right-aligned (3 chars * 6 px = 18 px)
// Line 2: status box left (see updateHeaderStatus) | page name right-aligned
static void drawHeader() {
  tft.fillRect(0, 0, 240, HEADER_H, C_HDR_BG);
  tft.setTextSize(1);
  tft.setTextColor(C_HDR_FG, C_HDR_BG);

  // Line 1 left: title
  tft.setCursor(2, 3);
  tft.print(F("DMX512 Monitor"));

  // Line 1 right: "X/3" (3 chars * 6 px = 18 px -> x = 220, 2 px margin)
  tft.setCursor(220, 3);
  tft.print(currentPage);
  tft.print(F("/3"));

  // Line 2 right: page name right-aligned (2 px margin)
  // "All channels" = 12 chars -> x = 166
  // "Ch. 1-256"    =  9 chars -> x = 184
  // "Ch. 257-512"  = 11 chars -> x = 172
  if (currentPage == 1) {
    tft.setCursor(166, 12);
    tft.print(F("All channels"));
  } else if (currentPage == 2) {
    tft.setCursor(184, 12);
    tft.print(F("Ch. 1-256"));
  } else {
    tft.setCursor(172, 12);
    tft.print(F("Ch. 257-512"));
  }
}

// ── Update the status area of the header ─────────────────────────────────────
// Line 2 left: coloured background (green/red) + white text " OK "/"FAIL"
// Region x=0..27 (4 chars * 6 px + 4 px margin), y=10..19 (lower half)
static void updateHeaderStatus() {
  const uint16_t boxColor = dmxOk ? C_OK : C_FAIL;
  tft.fillRect(0, 10, 28, 10, boxColor);
  tft.setTextSize(1);
  tft.setTextColor(C_HDR_FG, boxColor);
  tft.setCursor(2, 12);
  tft.print(dmxOk ? F(" OK ") : F("FAIL"));
}

// ── Page 1: column headers ───────────────────────────────────────────────────
// Draws column headers 0-15 using the 4x5 bitmap font.
// Each header cell is P1_CELL_W=14 px wide, 8 px tall (y = HEADER_H+1).
// 1 digit (0-9)  : glyph centred at cx=5 (4 px wide, centre of 14 px ≈ 5)
// 2 digits (10-15): left glyph cx=2, right glyph cx=7 (9 px, centre of 14 px ≈ 3)
static void drawP1ColHeaders() {
  tft.fillRect(0, HEADER_H, 240, COLHDR_H, C_COL_BG);
  tft.drawFastVLine(P1_ROWHDR_W - 1, HEADER_H, COLHDR_H, C_GRID);
  const uint8_t BUF_W = P1_CELL_W;   // 14
  const uint8_t BUF_H = 8;
  const uint16_t n    = (uint16_t)BUF_W * BUF_H;
  for (uint8_t c = 0; c < P1_COLS; c++) {
    for (uint16_t i = 0; i < n; i++) pxBuf[i] = C_COL_BG;
    if (c < 10) {
      blitChar(c,       5, 1, BUF_W, C_COL_FG, C_COL_BG);
    } else {
      blitChar(c / 10,  2, 1, BUF_W, C_COL_FG, C_COL_BG);
      blitChar(c % 10,  7, 1, BUF_W, C_COL_FG, C_COL_BG);
    }
    const uint16_t x = (uint16_t)P1_ROWHDR_W + (uint16_t)c * P1_CELL_W;
    tft.setAddrWindow(x, HEADER_H + 1, x + BUF_W - 1, HEADER_H + BUF_H);
    tft.pushColors(pxBuf, n, true);
  }
}

// ── Page 1: row headers (first DMX address of each row, right-aligned) ───────
// Same format as pages 2/3: background C_BG, text C_COL_FG.
// P1_ROWHDR_W=16 px | separator at x=15 | rx=11: last glyph at x=11..14
static void drawP1RowHeaders() {
  const uint8_t  rows  = (uint8_t)(DMX_CHANNELS / P1_COLS);
  const uint16_t dataH = (uint16_t)rows * P1_CELL_H;
  tft.fillRect(0, GRID_Y, P1_ROWHDR_W, dataH, C_BG);
  tft.drawFastVLine(P1_ROWHDR_W - 1, GRID_Y, dataH, C_GRID);
  for (uint8_t r = 0; r < rows; r++) {
    const uint16_t ry      = (uint16_t)GRID_Y + (uint16_t)r * P1_CELL_H;
    const uint16_t firstCh = (uint16_t)r * P1_COLS + 1;
    drawBitmapDecimal(firstCh, 11, ry + 2, C_COL_FG, C_BG);
  }
}

// ── Pages 2/3: background (grid, row and column headers) ────────────────────
static void drawP23Background(uint8_t page) {
  const uint16_t startIdx = p23StartIdx(page);
  const uint8_t  rows     = p23RowCount(page);
  const uint16_t dataH    = (uint16_t)rows * P23_CELL_H;

  // Column header row background and vertical separator
  tft.fillRect(0, HEADER_H, 240, COLHDR_H, C_COL_BG);
  tft.drawFastVLine(P23_ROWHDR_W, HEADER_H, COLHDR_H, C_GRID);

  // Column headers 0-9: single digit, centred in P23_CELL_W=20 px
  // Glyph width=4 px -> left edge at slot_x + (20-4)/2 = slot_x + 8
  // Glyph height=5 px -> top at HEADER_H + (COLHDR_H-5)/2 = HEADER_H + 2
  for (uint8_t c = 0; c < P23_COLS; c++) {
    const uint16_t cx = (uint16_t)P23_COL_X0 + (uint16_t)c * P23_SLOT;
    drawBitmapChar(c, cx + 8, HEADER_H + 2, C_COL_FG, C_COL_BG);
  }

  tft.fillRect(0, GRID_Y, 240, dataH, C_BG);

  tft.drawFastVLine(P23_ROWHDR_W, GRID_Y, dataH, C_GRID);
  for (uint8_t c = 0; c < P23_COLS; c++) {
    const uint16_t lx = (uint16_t)P23_COL_X0 + (uint16_t)c * P23_SLOT + P23_CELL_W;
    tft.drawFastVLine(lx, GRID_Y, dataH, C_GRID);
  }

  // Row headers: first channel number of each row, right-aligned.
  // Available width: P23_ROWHDR_W-1=26 px. Right edge of last glyph at x=24
  // (1 px margin before the separator). drawBitmapDecimal rx = last-glyph x-start = 21.
  // Glyph height=5 px, row inner height=9 px -> top at ry + (9-5)/2 = ry + 2.
  for (uint8_t r = 0; r < rows; r++) {
    const uint16_t ry      = (uint16_t)GRID_Y + (uint16_t)r * P23_CELL_H;
    const uint16_t firstCh = startIdx + (uint16_t)r * P23_COLS + 1;
    tft.drawFastHLine(0, ry + P23_CELL_H - 1, 238, C_GRID);
    tft.fillRect(0, ry, P23_ROWHDR_W, P23_CELL_H - 1, C_BG);
    drawBitmapDecimal(firstCh, 21, ry + 2, C_COL_FG, C_BG);
  }
}

// ── Pages 2/3: draw a single cell (fast, via pushColors) ────────────────────
// Value 0-255 in decimal, right-aligned, within a 20x9 px cell
// 1 digit: 1 glyph, 2 digits: 2 glyphs, 3 digits: 3 glyphs
// Glyph size 4x5 px, 1 px gap; right edge of last glyph at px+17
static void drawP23Cell(uint16_t pageChIdx, uint8_t val, bool highlight) {
  const uint8_t  row = (uint8_t)(pageChIdx / P23_COLS);
  const uint8_t  col = (uint8_t)(pageChIdx % P23_COLS);
  const uint16_t x   = (uint16_t)P23_COL_X0 + (uint16_t)col * P23_SLOT;
  const uint16_t y   = (uint16_t)GRID_Y + (uint16_t)row * P23_CELL_H;

  const uint16_t fg = highlight ? C_CHG_FG : valueColor(val);
  const uint16_t bg = highlight ? C_CHG_BG : C_BG;

  // Clear buffer (P23_PX_W=20, P23_PX_H=9)
  const uint16_t n = (uint16_t)P23_PX_W * P23_PX_H;
  for (uint16_t i = 0; i < n; i++) pxBuf[i] = bg;

  // Split decimal value into up to 3 glyphs and blit right-aligned
  // cx positions (start pixel, 4 px wide, 1 px gap):
  //   3 digits: cx = 2, 8, 14  (right edge 18 <= 19 px ✓)
  // Pixels 1..19 used for text, aligned to right edge
  // 1 digit:  cx = 14, cy = 2
  // 2 digits: cx1=8,  cx2=14, cy=2
  // 3 digits: cx1=2,  cx2=8,  cx3=14, cy=2
  const uint8_t cy = 2;
  if (val >= 100) {
    blitChar(val / 100,        2,  cy, P23_PX_W, fg, bg);
    blitChar((val / 10) % 10,  8,  cy, P23_PX_W, fg, bg);
    blitChar(val % 10,        14,  cy, P23_PX_W, fg, bg);
  } else if (val >= 10) {
    blitChar(val / 10,         8,  cy, P23_PX_W, fg, bg);
    blitChar(val % 10,        14,  cy, P23_PX_W, fg, bg);
  } else {
    blitChar(val,             14,  cy, P23_PX_W, fg, bg);
  }

  tft.setAddrWindow(x, y, x + P23_PX_W - 1, y + P23_PX_H - 1);
  tft.pushColors(pxBuf, n, true);
}

static void drawAllP23Cells(uint8_t page) {
  const uint16_t start = p23StartIdx(page);
  const uint16_t count = p23ChCount(page);
  for (uint16_t i = 0; i < count; i++) {
    drawP23Cell(i, snapshot[start + i], false);
  }
}

static void updateP23Cells(uint8_t page) {
  const uint16_t start = p23StartIdx(page);
  const uint16_t count = p23ChCount(page);
  for (uint16_t i = 0; i < count; i++) {
    const uint16_t si    = start + i;
    const bool     isNew = (snapshot[si] != prevSnapshot[si]);
    if (isNew) {
      flashTimer[si] = HIGHLIGHT_TICKS;
      drawP23Cell(i, snapshot[si], true);
    } else if (flashTimer[si] > 0) {
      flashTimer[si]--;
      if (flashTimer[si] == 0) drawP23Cell(i, snapshot[si], false);
    }
  }
}

// ── Full screen redraw ───────────────────────────────────────────────────────
static void fullRedraw() {
  tft.fillScreen(C_BG);
  drawHeader();
  updateHeaderStatus();
  if (currentPage == 1) {
    drawP1ColHeaders();
    drawP1RowHeaders();
    drawAllP1Cells();
  } else {
    drawP23Background(currentPage);
    drawAllP23Cells(currentPage);
  }
}

// ── Read DMX snapshot ────────────────────────────────────────────────────────
static void readDmxSnapshot() {
  for (uint16_t ch = 1; ch <= DMX_CHANNELS; ch++) {
    snapshot[ch - 1] = DMXSerial.read(ch);
  }
}

// ── Process touch input ──────────────────────────────────────────────────────
// Ghost-touch suppression: lastTouchMs is initialised so that no touch event
// is acted upon during the first TOUCH_BOOT_MS after reset.
// All four touch pins are reset to OUTPUT after getPoint() to prevent
// floating inputs from interfering with the 8-bit TFT bus.
static void handleTouch() {
  static uint32_t lastTouchMs = (uint32_t)(-TOUCH_BOOT_MS - TOUCH_COOLDOWN_MS);

  TSPoint p = ts.getPoint();
  // Reset all four touch pins (XM + YM were missing in an earlier revision)
  pinMode(TOUCH_XP, OUTPUT);
  pinMode(TOUCH_YP, OUTPUT);
  pinMode(TOUCH_XM, OUTPUT);
  pinMode(TOUCH_YM, OUTPUT);

  if (p.z < MINPRESSURE || p.z > MAXPRESSURE) return;
  if (millis() - lastTouchMs < TOUCH_COOLDOWN_MS) return;
  lastTouchMs = millis();

  const int16_t sx = (int16_t)map(p.x, TS_MINX, TS_MAXX, 0, 240);
  if (sx < 120) {
    currentPage = (currentPage > 1) ? currentPage - 1 : 3;
  } else {
    currentPage = (currentPage < 3) ? currentPage + 1 : 1;
  }
  fullRedraw();
}

// ── Serial output (only compiled when SERIAL_OUTPUT_ENABLE is defined) ───────
#ifdef SERIAL_OUTPUT_ENABLE

static void printPadded3(uint16_t value) {
  if (value < 100) Serial.print('0');
  if (value <  10) Serial.print('0');
  Serial.print(value);
}

static void printSeparator() {
  Serial.println(F("------------------------------------------------------------------------"));
}

static void printSerialHeader() {
  Serial.println();
  printSeparator();
  Serial.println(F("DMX512 Live Monitor | Arduino Mega 2560 | DMX on Serial1 (RX1 Pin 19)"));
  Serial.println(F("Matrix: row base + column offset (+00 ... +15)"));
  Serial.println(F("Example: row 033, column +05 -> channel 038"));
  printSeparator();
  Serial.print(F("Base |"));
  for (uint8_t c = 0; c < P1_COLS; c++) {
    Serial.print(F(" +"));
    if (c < 10) Serial.print('0');
    Serial.print(c);
  }
  Serial.println();
  printSeparator();
}

static void printStats() {
  uint16_t nonZero = 0;
  uint8_t  minVal  = 255;
  uint8_t  maxVal  = 0;
  for (uint16_t i = 0; i < DMX_CHANNELS; i++) {
    if (snapshot[i])          nonZero++;
    if (snapshot[i] < minVal) minVal = snapshot[i];
    if (snapshot[i] > maxVal) maxVal = snapshot[i];
  }
  Serial.print(F("Time "));          Serial.print(millis());
  Serial.print(F(" | Non-zero: ")); Serial.print(nonZero);
  Serial.print(F(" | Min: "));      Serial.print(minVal);
  Serial.print(F(" | Max: "));      Serial.println(maxVal);
  printSeparator();
}

static void printMatrix() {
  for (uint16_t base = 0; base < DMX_CHANNELS; base += P1_COLS) {
    printPadded3(base + 1);
    Serial.print(F("  |"));
    for (uint8_t c = 0; c < P1_COLS; c++) {
      Serial.print(' ');
      printPadded3(snapshot[base + c]);
    }
    Serial.println();
  }
  printSeparator();
}

static void doSerialOutput() {
  printSerialHeader();
  printStats();
  printMatrix();
}

#endif  // SERIAL_OUTPUT_ENABLE

// ── Setup ────────────────────────────────────────────────────────────────────
void setup() {
#ifdef SERIAL_OUTPUT_ENABLE
  Serial.begin(DEBUG_BAUD);
  delay(300);
  Serial.println(F("Starting DMX512 Live Monitor ..."));
  Serial.println(F("Expecting DMX on Serial1 / RX1 (Pin 19)"));
  Serial.println(F("USB console running on Serial0"));
  Serial.println(F("No values? Check A/B polarity, RE/DE to GND, 120 ohm termination."));
#endif

  memset(snapshot,     0, sizeof(snapshot));
  memset(prevSnapshot, 0, sizeof(prevSnapshot));
  memset(flashTimer,   0, sizeof(flashTimer));

  // Start DMX receiver (requires "#define DMX_USE_PORT1" in DMXSerial_avr.h)
  DMXSerial.init(DMXReceiver);

  // Initialise TFT; force ILI9341 ID for clone shields that return 0xD3D3 or 0x0000
  uint16_t tftId = tft.readID();
  if (tftId == 0xD3D3 || tftId == 0x0000) tftId = 0x9341;
  tft.begin(tftId);
  tft.setRotation(0);   // portrait: 240 x 320 px
  tft.setTextSize(1);

  fullRedraw();
}

// ── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
  static uint32_t lastRefresh = 0;

  handleTouch();

  if (millis() - lastRefresh < REFRESH_MS) return;
  lastRefresh = millis();

  memcpy(prevSnapshot, snapshot, sizeof(snapshot));
  readDmxSnapshot();

  const bool newDmxOk = (DMXSerial.noDataSince() < DMX_TIMEOUT_MS);
  if (newDmxOk != dmxOk) {
    dmxOk = newDmxOk;
    updateHeaderStatus();
  }

  if (currentPage == 1) {
    updateP1Cells();
  } else {
    updateP23Cells(currentPage);
  }

#ifdef SERIAL_OUTPUT_ENABLE
  doSerialOutput();
#endif
}
