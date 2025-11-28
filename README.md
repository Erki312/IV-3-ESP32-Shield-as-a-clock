# Axiris IV-3 ESP32-S3 Internet Clock

![Clock on the desk](Pictures/Clock.JPG)

This project is a modernized version of the Axiris IV-3 clock from Elektor magazine (issue 2015).  
The original design used an AVR-based Arduino and a serial command interface.  

This version replaces the controller with an ESP32-S3 and adds:

- Automatic time synchronization via SNTP/NTP over Wi-Fi  
- A built-in web interface for Wi-Fi and time zone configuration  
- A fallback access point for initial setup  
- The original IV-3 tube display behavior (multiplexing, date/time display, LED dimming) preserved in spirit

The original Arduino sketch is included as a reference in a separate folder and is not written by me (see `third_party/axiris_iv3_original/NOTICE`).

---

## Features

- ESP32-S3 based controller (ESP-IDF, no Arduino core required)
- Drives the IV-3 tube shield directly using GPIO + GPTimer
- NTP time sync using `esp_sntp` once the clock is online
- Wi-Fi Station mode (connects to your home network)
- Wi-Fi Access Point setup mode if no Wi-Fi is configured or STA connection fails  
  - SSID: `NixieClock-Setup`  
  - Default password: `12345678`
- Built-in HTTP web UI
  - Status page (`/`): mode, Wi-Fi info, time, IP address
  - Config page (`/config`): Wi-Fi SSID, password, time zone (dropdown with common TZ presets)
- Time zone support via POSIX TZ strings, stored in NVS
- Date display like the original firmware:
  - Normally shows HH:MM
  - Between 50 and 54 seconds, shows DD.MM

---

## Hardware

- Axiris IV-3 clock shield (Elektor design)
- ESP32-S3 dev board in Arduino/UNO form factor

**Important: Analog outputs A0 to A3 need to be able to be configured as outputs.**
I used the following from AliExpress [ESP32-S3-N16R8](https://de.aliexpress.com/item/1005001953904745.html?spm=a2g0o.cart.0.0.72f84ae4sPUZhW&mp=1&pdp_npi=5%40dis%21EUR%21EUR%206.25%21EUR%204.69%21%21EUR%204.69%21%21%21%402103835c17643342033767090ecf60%2112000048046652793%21ct%21DE%21-1%21%211%210&gatewayAdapt=glo2deu).


### Pin mapping (ESP32-S3)

The project assumes the following mapping:

- Grids:
  - `PIN_GRID0` = GPIO 2   (A0)  
  - `PIN_GRID1` = GPIO 1   (A1)  
  - `PIN_GRID2` = GPIO 7   (A2)  
  - `PIN_GRID3` = GPIO 6   (A3)  

- Segments:
  - `PIN_SEG_A` = GPIO 18  (D2)  
  - `PIN_SEG_B` = GPIO 17  (D3)  
  - `PIN_SEG_C` = GPIO 19  (D4)  
  - `PIN_SEG_D` = GPIO 20  (D5)  
  - `PIN_SEG_E` = GPIO 3   (D6)  
  - `PIN_SEG_F` = GPIO 14  (D7)  
  - `PIN_SEG_G` = GPIO 21  (D8)  
  - `PIN_DOT`   = GPIO 46  (D9)  

- LEDs under the tubes:
  - `PIN_LEDS`  = GPIO 10  (D10)

If your ESP32-S3 board uses different pins, adjust the definitions at the top of the source file accordingly.

---

## Software / Toolchain

This project is written for the **ESP-IDF** (tested with ESP-IDF 5.x).  

You will need:

- ESP-IDF installed and set up (`idf.py` available in your shell)
- An ESP32-S3 toolchain
- A serial connection to the board

---

## Building and Flashing

Typical ESP-IDF workflow:

```bash
# 1. Set target
idf.py set-target esp32s3

# 2. Configure project (if needed)
idf.py menuconfig

# 3. Build
idf.py build

# 4. Flash and monitor
idf.py flash monitor
