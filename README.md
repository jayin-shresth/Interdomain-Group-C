TEAM C AUTONOMOUS PIPE EXPLORATION ROBOT
# 🤖 Pipe Exploration Robot
### *Interdomain Group-C Project*

---

## 🧠 Overview
This project presents an **autonomous pipe exploration robot** designed to navigate through confined pipeline systems.  
It intelligently explores unknown paths using a **breadcrumb tracking system**, a **greedy algorithm** for optimal path selection, and a **reverse navigation** method to safely return to its starting point.

---

## ⚙️ Features
- 🚀 **Autonomous Navigation** – Moves through pipes without manual control.
- 🍞 **Breadcrumb Tracking** – Marks visited nodes to prevent redundant traversal.
- 🧭 **Greedy Algorithm** – Chooses the nearest unexplored node to ensure efficient coverage.
- 🔁 **Reverse Navigation** – Uses stored breadcrumbs to return safely.
- 📡 **Sensor-Based Detection** – Ultrasonic and IR sensors for obstacle avoidance and turn detection.
- 💾 **Data Logging** – Records exploration paths and sensor data for post-run analysis.

---

## 🧩 System Architecture
| Component | Function |
|------------|-----------|
| **MCU** | Core control unit (Arduino/ESP32) handling movement and sensor input |
| **Sensors** | Ultrasonic and IR sensors for obstacle detection |
| **Motor Driver** | Controls locomotion and turning |
| **Communication Module** | LoRa/Serial for telemetry and debugging |
| **Power Supply** | Li-ion battery pack powering the system |

---

## 🧮 Algorithms Used
### 1. **Breadcrumb Algorithm**
Stores each node visited by the robot as a “breadcrumb.” Prevents revisiting the same location and enables return path reconstruction.

### 2. **Greedy Algorithm**
At every junction, the robot selects the nearest unexplored path, maximizing coverage efficiency.

### 3. **Reverse Navigation**
Retraces the path using stored breadcrumbs in reverse order to safely return to the start point.

---

## 🧠 Working Principle
1. The robot begins exploration from a **base node**.
2. Sensors detect the pipe’s geometry and available directions.
3. Each move is logged using **breadcrumbs**.
4. When multiple paths are detected, the **greedy algorithm** decides the optimal next path.
5. Once the exploration completes or a dead end is found, **reverse navigation** guides the robot back.

---

## 🔧 Hardware Requirements
- Arduino Uno / ESP32
- Motor driver (L298N)
- Ultrasonic sensor (HC-SR04)
- LoRa / Serial module (optional)
- Li-ion battery pack

---

## 💻 Software Requirements
- Arduino IDE / PlatformIO
- C / C++ firmware for control logic
- Python (for simulation or data visualization, optional)

---


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

### Rover Firmware — `rover_lora.ino`
- Sends **Heartbeat packets** every 3 seconds  
- Stores mission logs in memory (no SD required)  
- On receiving `CMD:DUMP`, transmits all logs to the base

### Base Firmware — `base_lora.ino`
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


/communication/
├── rover_lora/
│ ├── rover_lora.ino # Main rover code with LoRa communication and improvements
│ ├── comm_protocol.h # Packet structs, CRC/HMAC helpers for secure communication
│ ├── sensors.h / sensors.cpp # Sensor read functions (pH, turbidity, battery)
│ ├── logger.h / logger.cpp # SD logging wrapper for local data storage
│ └── config.h # Constants (device ID, spreading factors, intervals)
│
├── base_lora/
│ ├── base_lora.ino # Base station code with parsing and file logging
│ └── gateway_serial_logger.py # Python script to read base serial and forward data to dashboard
│
├── dashboard/
│ ├── lora_dashboard.py # Real-time data plotting and CSV logging
│ └── dashboard_readme.md # Dashboard usage instructions
│
## 📸 Project Images

<p align="center">
  <img src="/home/swetank/Interdomain-Group-C/images/heartbeat_recieves.png" alt="heartbeat recieved via mqtt" width="450"><br>
  <em>Figure 1: heartbeat recieved via mqtt</em>
</p>

<p align="center">
  <img src="/home/swetank/Interdomain-Group-C/images/heartbeat_sim.png" alt="heartbeat simulation" width="500"><br>
  <em>Figure 2: heartbeat simulation</em>
</p>

