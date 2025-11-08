#!/usr/bin/env python3
"""
lora_dashboard.py
-----------------
LoRa Rover Telemetry & Sample Analysis Dashboard

✅ Compatible with Streamlit 1.32+
✅ Works with fake_lora_publisher.py
✅ Displays Heartbeat, Analysis, and RSSI charts live
"""

import streamlit as st
import threading, time, json
from collections import deque
import paho.mqtt.client as mqtt
import pandas as pd

# -----------------------------
# Streamlit Page Configuration
# -----------------------------
st.set_page_config(layout='wide', page_title='LoRa Rover Dashboard')
st.title(" LoRa Rover Telemetry Dashboard")

# -----------------------------
# Global data holders
# -----------------------------
MAXLEN = 300
rssi_deque = deque(maxlen=MAXLEN)
seq_deque = deque(maxlen=MAXLEN)
latest_hb = {}
latest_analysis = {}

# -----------------------------
# Sidebar Settings
# -----------------------------
st.sidebar.header(" MQTT Configuration")
MQTT_BROKER = st.sidebar.text_input("Broker Address", "localhost")
MQTT_TOPIC = st.sidebar.text_input("MQTT Topic", "pipe/telemetry")
MQTT_PORT = 1883
refresh_rate = st.sidebar.slider("Refresh interval (seconds)", 1, 10, 2)

st.sidebar.markdown("---")
st.sidebar.info(" Keep fake_lora_publisher.py running.\n"
                 "This dashboard auto-refreshes and updates from MQTT messages.")

# -----------------------------
# MQTT Callback Functions
# -----------------------------
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f" Connected to MQTT broker {MQTT_BROKER}:{MQTT_PORT}")
        client.subscribe(MQTT_TOPIC)
        print(f" Subscribed to topic: {MQTT_TOPIC}")
    else:
        print(" MQTT Connection failed with code:", rc)

def on_message(client, userdata, msg):
    global latest_hb, latest_analysis
    try:
        payload = json.loads(msg.payload.decode())
        msg_type = payload.get("type")

        if msg_type == "HB":  # Heartbeat
            latest_hb = payload
            rssi_deque.append(payload.get("rssi", 0))
            seq_deque.append(payload.get("seq", 0))

        elif msg_type == "ANAL":  # Analysis result
            latest_analysis = payload

        # Print received data for debug
        print("📩", payload)

    except Exception as e:
        print("⚠️ Error parsing message:", e)

# -----------------------------
# MQTT Background Thread
# -----------------------------
def mqtt_thread():
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.loop_forever()
    except Exception as e:
        print(f"⚠️ MQTT Connection Error: {e}")

# Start the MQTT listener in the background
threading.Thread(target=mqtt_thread, daemon=True).start()

# -----------------------------
# Dashboard Layout Function
# -----------------------------
def draw_dashboard():
    st.subheader("📡 Live Rover Telemetry")

    col1, col2 = st.columns(2)

    # Heartbeat Panel
    with col1:
        st.markdown("### ❤️ Last Heartbeat")
        if latest_hb:
            st.markdown(
                f"""
                **Timestamp:** {latest_hb.get('ts', 'N/A')}  
                **Battery:** {latest_hb.get('batt', '?')}%  
                **RSSI:** {latest_hb.get('rssi', '?')} dBm  
                **Phase:** {latest_hb.get('phase', 'N/A')}  
                **Seq:** {latest_hb.get('seq', 'N/A')}
                """
            )
        else:
            st.info("Waiting for heartbeat packets...")

    # Analysis Panel
    with col2:
        st.markdown("### 🧪 Last Sample Analysis")
        if latest_analysis:
            st.markdown(
                f"""
                **Timestamp:** {latest_analysis.get('ts', 'N/A')}  
                **pH:** {latest_analysis.get('ph', '?')}  
                **Turbidity:** {latest_analysis.get('turb', '?')} NTU  
                **ATP:** {latest_analysis.get('atp', '?')}
                """
            )
        else:
            st.info("No analysis data received yet.")

    # RSSI Chart
    st.markdown("### 📉 RSSI Signal Strength Over Time")
    if len(rssi_deque) > 1:
        df = pd.DataFrame({
            "Seq": list(seq_deque),
            "RSSI": list(rssi_deque)
        })
        st.line_chart(df, x="Seq", y="RSSI", height=300, use_container_width=True)
    else:
        st.warning("Waiting for RSSI values...")

# -----------------------------
# Auto-refresh loop
# -----------------------------
draw_dashboard()
st.caption(f"🔄 Auto-refreshing every {refresh_rate} seconds...")
time.sleep(refresh_rate)
st.rerun()

