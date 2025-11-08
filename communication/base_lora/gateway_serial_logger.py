#!/usr/bin/env python3
"""
gateway_serial_logger.py
----------------------------------------
LoRa Gateway Logger for Pipe Rover Project

- Reads serial output from Base ESP32
- Handles two message types:
   [HB] seq,batt,rssi,ts    -> Heartbeat telemetry
   [LOG] chunkId:payload    -> Mission log data
- Publishes Heartbeat telemetry to MQTT
- Saves telemetry and mission logs to CSV files
----------------------------------------
"""

import argparse
import serial
import time
import json
import os
import re
from datetime import datetime
import paho.mqtt.client as mqtt

# ---------- ARGUMENTS ----------
parser = argparse.ArgumentParser(description="LoRa Gateway Serial Logger")
parser.add_argument("--port", required=True, help="Serial port (e.g., /dev/ttyUSB0)")
parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default 115200)")
parser.add_argument("--mqtt", default="localhost", help="MQTT broker host")
parser.add_argument("--topic", default="pipe/telemetry", help="MQTT telemetry topic")
args = parser.parse_args()

# ---------- SETUP PATHS ----------
os.makedirs("logs", exist_ok=True)
telemetry_csv = os.path.join("logs", "telemetry_log.csv")
mission_csv = os.path.join("logs", "received_mission_log.csv")

# ---------- MQTT CLIENT ----------
client = mqtt.Client()
client.connect(args.mqtt, 1883, 60)
client.loop_start()

# ---------- SERIAL ----------
def open_serial():
    try:
        return serial.Serial(args.port, args.baud, timeout=1)
    except Exception as e:
        print(f"[{datetime.now().isoformat()}] ❌ Serial open failed: {e}")
        return None

ser = open_serial()
time.sleep(1)

# ---------- REGEX MATCHERS ----------
hb_re = re.compile(r'^\[HB\]\s*(\d+),\s*(\d+),\s*(-?\d+),\s*(\d+)\s*$')
log_re = re.compile(r'^\[LOG\]\s*(.*)$', re.DOTALL)

# ---------- CSV HEADERS ----------
if not os.path.exists(telemetry_csv):
    with open(telemetry_csv, "w") as f:
        f.write("timestamp,seq,batt,rssi\n")

if not os.path.exists(mission_csv):
    with open(mission_csv, "w") as f:
        f.write("seq,phase,x,y,heading,ph,turb,batt,ts\n")

# ---------- UTILITY ----------
def now_str():
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")

def append_mission_chunk(payload):
    """Append LOG payloads to mission log file"""
    with open(mission_csv, "a") as f:
        f.write(payload.strip() + "\n")
    print(f"[{now_str()}] 📄 Added mission log chunk ({len(payload)} bytes)")

def append_telemetry(seq, batt, rssi):
    """Append heartbeat telemetry"""
    with open(telemetry_csv, "a") as f:
        f.write(f"{datetime.utcnow().isoformat()},{seq},{batt},{rssi}\n")

# ---------- MAIN LOOP ----------
print(f"[{now_str()}]  Gateway started")
print(f"Serial: {args.port} @ {args.baud}")
print(f"MQTT Broker: {args.mqtt}")
print(f"Logging to: {telemetry_csv} and {mission_csv}\n")

while True:
    try:
        if ser is None or not ser.is_open:
            print(f"[{now_str()}]  Reconnecting serial...")
            time.sleep(2)
            ser = open_serial()
            continue

        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            continue

        # ---- HEARTBEAT ----
        m = hb_re.match(line)
        if m:
            seq = int(m.group(1))
            batt = int(m.group(2))
            rssi = int(m.group(3))
            ts = int(m.group(4))

            hb_data = {
                "type": "HB",
                "seq": seq,
                "batt": batt,
                "rssi": rssi,
                "ts": ts,
                "iso_time": datetime.utcnow().isoformat() + "Z"
            }

            # MQTT publish
            client.publish(args.topic, json.dumps(hb_data))
            append_telemetry(seq, batt, rssi)
            print(f"[{now_str()}]  HB seq={seq}, batt={batt}%, rssi={rssi}")
            continue

        # ---- LOG CHUNK ----
        m2 = log_re.match(line)
        if m2:
            payload = m2.group(1)
            # Remove chunkId if present
            if ":" in payload:
                _, data = payload.split(":", 1)
            else:
                data = payload
            append_mission_chunk(data)
            continue

        # ---- OTHER LINES ----
        if line.startswith("[RAW]"):
            print(f"[{now_str()}]  RAW:", line)
        else:
            print(f"[{now_str()}]  {line}")

    except KeyboardInterrupt:
        print(f"\n[{now_str()}] 🛑 Stopping gateway...")
        break
    except Exception as e:
        print(f"[{now_str()}] ⚠️ Error: {e}")
        time.sleep(1)

# ---------- CLEANUP ----------
client.loop_stop()
client.disconnect()
if ser and ser.is_open:
    ser.close()
print(f"[{now_str()}] ✅ Gateway exited cleanly.")

