# Adafruit LoRa Bonnet RX App

This repository contains a C program for the Adafruit RFM95W LoRa Radio Bonnet (PID 4074) on a Raspberry Pi 4. \
It opens the bonnet’s RFM95 (SPI), SSD1306 OLED (I²C), and buttons (via libgpiod v2) and focuses on receiving LoRa packets while still allowing quick test transmissions through Button A.

## Hardware

- Raspberry Pi 4 Model B (Raspberry Pi OS bookworm or later)
- Adafruit LoRa Radio Bonnet with OLED – RFM95W (Product 4074)
- Optional: matching LoRa peer for transmitting test packets

## Pi Setup

1. Enable SPI and I²C: `sudo raspi-config` → *Interface Options* → enable both, then reboot.
2. Install dependencies:
   ```bash
   sudo apt update
   sudo apt install -y build-essential cmake libgpiod-dev
   ```
3. Confirm devices exist: `ls /dev/spidev0.*` and `ls /dev/i2c-1`.

## Build

```bash
cmake -S . -B build
cmake --build build
```

This produces `build/adafruit_lora_hat`.

## Run

```bash
sudo ./build/adafruit_lora_hat
```

Running as root is required for access to `/dev/spidev0.1`, `/dev/i2c-1`, and `/dev/gpiochip0`.

## Controls

- **Button A** – send a short “Pi TX #N” message (useful to verify the RF path).
- **Button B** – toggle OLED between the last payload text and RSSI/SNR stats.
- **Button C** – clear the OLED and wait for the next packet.

Whenever a packet is received:

- The payload is sanitized and printed to the terminal with timestamp, byte count, RSSI, and SNR.
- The OLED shows the payload (or stats, depending on mode).

## Configuration

Relevant defaults are near the top of `adafruit_lora_hat.c`:

- `RADIO_FREQ_MHZ` – default 915 MHz; change to 868 MHz or others to match regional regulations.
- `TX_POWER_DBM` – transmit power (obey local RF rules).
- `radio_set_modem()` call – adjust bandwidth, spreading factor, and coding rate if you need different LoRa parameters.

## Notes

- The code uses libgpiod v2’s request/config API; earlier headers won’t work.
- Ensure the bonnet is wired to CE1 (`/dev/spidev0.1`). If you use CE0, change `SPI_DEV` accordingly.
- For better performance you can wire DIO0 to an interrupt-capable GPIO and adapt the loop, but the current version polls.
