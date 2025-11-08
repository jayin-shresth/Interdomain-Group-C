TEAM C AUTONOMOUS PIPE EXPLORATION ROBOT

#  Pipe Exploration Robot — Communication Subsystem

This document explains the **LoRa-based communication system** for the Pipe Exploration Robot project, including setup, wiring, firmware, and testing instructions.

---

## Overview

The communication subsystem enables **two-way data exchange** between:
- The **Rover** (inside the pipe)
- The **Base Station** (outside, near the operator)
- The **Laptop Gateway + Dashboard** (for telemetry visualization)

It uses **LoRa RA-02 (SX1278)** modules for long-range, low-power communication.

---

##  System Architecture

┌────────────────────────────────────────────────────────────┐
│  Laptop / PC │
│ ┌──────────────────────────────────────────────────────┐ │
│ │ gateway_serial_logger.py → MQTT → Streamlit Dashboard│ │
│ └──────────────────────────────────────────────────────┘ │
│ ↑ Serial USB │
│ │ │
│ Base Arduino + LoRa #1 (433 MHz) │
│ │ │
│ ~~~~~~~~~~ LoRa Link ~~~~~~~~~~ │
│ │ │
│ Rover Arduino/ESP32 + LoRa #2 │
│ │ │
│ Sensors / MCU / Navigation System │
└────────────────────────────────────────────────────────────┘

---

## 🧰 Hardware Used

| Component | Quantity | Description |
|------------|-----------|-------------|
| Arduino UNO / Nano | 2 | One for Rover, one for Base Station |
| LoRa RA-02 (SX1278) | 2 | 433 MHz long-range transceivers |
| Jumper wires | – | SPI + power connections |
| Laptop / PC | 1 | Runs Python gateway and dashboard |

---

## 🔌 Wiring Diagram (for each LoRa + Arduino)

| LoRa RA-02 Pin | Arduino Pin | Description |
|----------------|--------------|-------------|
| VCC | 3.3V |  Use 3.3 V only |
| GND | GND | Common ground |
| SCK | D13 | SPI Clock |
| MISO | D12 | SPI MISO |
| MOSI | D11 | SPI MOSI |
| NSS | D10 | Chip Select |
| RST | D9 | Reset |
| DIO0 | D2 | Interrupt |


---

## Firmware

### Rover Firmware — `rover_lora_nosd_arduino.ino`
- Sends **Heartbeat packets** every 3 seconds  
- Stores mission logs in memory (no SD required)  
- On receiving `CMD:DUMP`, transmits all logs to the base

### Base Firmware — `base_lora_arduino.ino`
- Receives Heartbeats and Log packets  
- Forwards everything to the Laptop over Serial  
- Accepts commands from Serial Monitor (like `dump`) and transmits via LoRa

---

##  Gateway Script — `gateway_serial_logger.py`
Runs on your laptop:
- Reads Serial data from the Base Station Arduino
- Parses:
  - `[HB]` → Heartbeat data
  - `[LOG]` → Mission log data
- Saves CSV files in `logs/` folder:

## project folder structure
communication/
├── base_lora_arduino.ino
├── rover_lora_nosd_arduino.ino
├── gateway_serial_logger.py
├── lora_dashboard.py
├── logs/
│   ├── telemetry_log.csv
│   └── received_mission_log.csv
└── README_Communication.md



