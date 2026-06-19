# Clock Radio

Arduino-based FM clock radio using the **Arduino Micro** and **CJMCU-470 (Si4703)** FM radio module.

## Hardware

| Component | Details |
|-----------|---------|
| Microcontroller | Arduino Micro (ATmega32U4) |
| FM Radio Module | CJMCU-470 (Si4703) |
| Interface | I²C |

## Wiring

| CJMCU-470 Pin | Arduino Micro Pin |
|---------------|-------------------|
| VCC | 3.3V |
| GND | GND |
| SDA | SDA (D2) |
| SCL | SCL (D3) |
| RST | D5 |

## Current Sketch

Tunes to **91.8 MHz** at maximum volume and reports signal info via serial (9600 baud):

- Frequency confirmation
- RSSI (signal strength in dBuV)
- Stereo / mono indicator

## Setup

1. Install the [Arduino Community Edition](https://marketplace.visualstudio.com/items?itemName=vscode-arduino.vscode-arduino-community) VS Code extension
2. Open this folder in VS Code — board, port and sketch are pre-configured in `.vscode/arduino.json`
3. Upload via `Arduino: Upload` in the command palette
