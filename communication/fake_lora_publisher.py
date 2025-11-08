#!/usr/bin/env python3
"""
fake_lora_publisher.py
Simulates LoRa rover telemetry and analysis packets via MQTT
"""
import json, time, random
import paho.mqtt.client as mqtt
from datetime import datetime

BROKER = "localhost"
TOPIC = "pipe/telemetry"

client = mqtt.Client()
client.connect("localhost", 1883, 60)


seq = 1
phase_list = ["start", "navigate", "sampling", "return"]

print("🚀 Starting fake LoRa telemetry publisher...")
print(f"Publishing to MQTT broker {BROKER}, topic {TOPIC}")

try:
    while True:
        # --- Heartbeat packet ---
        hb = {
            "type": "HB",
            "ts": datetime.utcnow().isoformat() + "Z",
            "device": "0x0101",
            "seq": seq,
            "batt": random.randint(70, 100),
            "rssi": random.randint(-85, -60),
            "phase": random.choice(phase_list),
        }
        client.publish(TOPIC, json.dumps(hb))
        print("[HB]", hb)

        # Every 5 heartbeats, send a fake analysis packet
        if seq % 5 == 0:
            analysis = {
                "type": "ANAL",
                "ts": datetime.utcnow().isoformat() + "Z",
                "device": "0x0101",
                "seq": seq,
                "ph": round(random.uniform(6.5, 7.5), 2),
                "turb": random.randint(40, 90),      # NTU
                "atp": random.randint(100, 400),     # arbitrary bio marker
                "flags": 0,
            }
            client.publish(TOPIC, json.dumps(analysis))
            print("[ANAL]", analysis)

        seq += 1
        time.sleep(2)

except KeyboardInterrupt:
    print("\n🛑 Simulation stopped.")
    client.disconnect()
