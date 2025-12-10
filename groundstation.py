#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# Raspberry Pi 4 + Adafruit RFM95W LoRa Radio Bonnet (PID 4074)
# - Transmits a short packet every 5 seconds
# - Shows status on the 128x32 OLED
# - Button A sends an immediate packet; Button B toggles fast/slow rate

import time
import socket
import busio
import board
from digitalio import DigitalInOut, Direction, Pull
import adafruit_ssd1306
import adafruit_rfm9x

# ----------------------------
# Buttons on the Bonnet
# ----------------------------
btnA = DigitalInOut(board.D5)   # left
btnA.direction = Direction.INPUT
btnA.pull = Pull.UP

btnB = DigitalInOut(board.D6)   # middle
btnB.direction = Direction.INPUT
btnB.pull = Pull.UP

btnC = DigitalInOut(board.D12)  # right (unused here)
btnC.direction = Direction.INPUT
btnC.pull = Pull.UP

# ----------------------------
# OLED (SSD1306 128x32 on I2C)
# ----------------------------
i2c = busio.I2C(board.SCL, board.SDA)
oled_reset = DigitalInOut(board.D4)  # Bonnet ties OLED RST to GPIO4
display = adafruit_ssd1306.SSD1306_I2C(128, 32, i2c, reset=oled_reset)
display.fill(0); display.show()

# Basic text helper
def oled_lines(lines):
    display.fill(0)
    y = 0
    for s in lines[:4]:  # 4 rows of 8px text on 32px-high OLED
        display.text(str(s)[:21], 0, y, 1)  # 21 chars-ish across
        y += 8
    display.show()

# ----------------------------
# RFM95 LoRa radio (SPI + control pins)
# ----------------------------
CS = DigitalInOut(board.CE1)     # Chip Select
RESET = DigitalInOut(board.D25)  # Reset
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

# Set your band here (US: 915.0 MHz; EU: 868.0 MHz; 433 region: 433.0 MHz)
RADIO_FREQ_MHZ = 915.0

oled_lines(["Init RFM95...", f"Freq {RADIO_FREQ_MHZ} MHz", ""])
try:
    rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ)
except RuntimeError as e:
    oled_lines(["RFM95 ERROR", "Check wiring/CE1", str(e)[:21]])
    raise

# Optional radio settings
rfm9x.tx_power = 20          # dBm (max 23; obey local regulations!)
rfm9x.signal_bandwidth = 125000  # Hz (typical)
rfm9x.coding_rate = 5
rfm9x.spreading_factor = 7

# ----------------------------
# Utility: best-effort IP display
# ----------------------------
def get_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0.1)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "no net"

ip = get_ip()

# ----------------------------
# Main loop
# ----------------------------
counter = 0
fast = False
last_tx = 0.0
period_slow = 5.0
period_fast = 1.0

oled_lines(["RFM95 ready", f"IP {ip}", "A:TX  B:rate"])

while True:
    now = time.monotonic()

    # Button B toggles rate
    if not btnB.value:
        fast = not fast
        oled_lines([f"Rate: {'FAST' if fast else 'SLOW'}", f"IP {ip}", "A:TX  B:rate"])
        time.sleep(0.25)  # debounce

    # Button A forces immediate TX
    do_tx = False
    if not btnA.value:
        do_tx = True
        time.sleep(0.15)  # debounce

    # Periodic TX
    period = period_fast if fast else period_slow
    if (now - last_tx) >= period:
        do_tx = True

    if do_tx:
        payload = f"Pi4 hello #{counter}"
        try:
            rfm9x.send(bytes(payload, "utf-8"))
            last_tx = now
            counter += 1
            oled_lines(["TX OK", payload, f"Pwr {rfm9x.tx_power} dBm"])
        except Exception as e:
            oled_lines(["TX ERROR", str(e)[:21]])
        # brief status dwell
        time.sleep(0.2)

    # (Optional) listen briefly for replies and show RSSI
    packet = rfm9x.receive(timeout=0.01)
    if packet is not None:
        try:
            msg = packet.decode("utf-8", errors="ignore")
        except Exception:
            msg = str(packet)
        oled_lines([f"RX {len(packet)}B RSSI {rfm9x.last_rssi}dB", msg[:21]])

    # Idle status heartbeat line every ~second
    if int(now) % 2 == 0:
        pass

    time.sleep(0.01)
