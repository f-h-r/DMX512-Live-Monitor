# Arduino DMX512 Live Monitor

Live monitor for DMX512 signals with a 2.4" TFT touch display (ILI9341, 240×320, 8-bit parallel).  
Displays all 512 channels across three touch-navigable pages.

---

## Sketches

### `dmx-debugger/dmx-debugger.ino` — Arduino Mega 2560

| | |
|---|---|
| **Board** | Arduino Mega 2560 (ATMega2560) |
| **DMX input** | Serial1 / RX1 (Pin 19) via RS485 transceiver (RE/DE tied to GND) |
| **SRAM usage** | ~2681 bytes (of 8192) |

DMX is received on Serial1, leaving Serial0 free for optional USB debug output.  
Changed channels are highlighted in yellow for 3 refresh frames (`HIGHLIGHT_TICKS = 3`).  
Serial debug output can be enabled by uncommenting `#define SERIAL_OUTPUT_ENABLE`.

---

### `dmx-debugger-uno/dmx-debugger-uno.ino` — Arduino Uno (ATMega328)

| | |
|---|---|
| **Board** | Arduino Uno (ATMega328P) |
| **DMX input** | Serial0 / RX0 (Pin 0) via RS485 transceiver (RE/DE tied to GND) |
| **SRAM usage** | ~1657 bytes (of 2048) |

Adapted from the Mega version to fit within the Uno's 2 KB SRAM budget.  
Because Serial0 is shared with USB, the RS485 transceiver must be disconnected before uploading.  
Change highlighting and serial debug output are removed to save memory.  
All 512 channel cells are redrawn on every refresh cycle (250 ms).

---

## Display pages

| Page | Content |
|------|---------|
| **1** | All 512 channels as a bar chart (16 columns × 32 rows) |
| **2** | Channels 1–256 as decimal values (10 columns × 26 rows) |
| **3** | Channels 257–512 as decimal values (10 columns × 26 rows) |

Touch the **left half** of the screen to go to the previous page, the **right half** for the next page.

---

## Required libraries

| Library | Author |
|---------|--------|
| DMXSerial | Matthias Hertel |
| MCUFRIEND_kbv | David Prentice |
| Adafruit GFX Library | Adafruit Industries |
| Adafruit BusIO | Adafruit Industries |
| TouchScreen | Adafruit Industries |

Install all libraries via the Arduino IDE Library Manager.  
For the **Mega** version, uncomment `#define DMX_USE_PORT1` in `DMXSerial_avr.h` to route DMX to Serial1.  
The **Uno** version uses Serial0 by default — no changes to the library are needed.
