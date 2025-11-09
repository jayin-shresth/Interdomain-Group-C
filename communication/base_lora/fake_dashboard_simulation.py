import streamlit as st
import time
import random
from collections import deque

st.set_page_config(layout='wide', page_title='LoRa Simulation Dashboard')

rssi_data = deque(maxlen=100)
seq = 0
placeholder = st.empty()

while True:
    seq += 1
    rssi = random.randint(-90, -40)
    batt = random.randint(70, 100)
    rssi_data.append(rssi)

    with placeholder.container():
        st.subheader(f"📡 Simulated LoRa RSSI: {rssi} dBm | 🔋 Battery: {batt}% | Seq: {seq}")
        st.line_chart(list(rssi_data))

    time.sleep(1)
