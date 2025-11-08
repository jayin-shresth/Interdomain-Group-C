#!/usr/bin/env python3


import argparse
import csv
import json
import time
import threading
import serial
import paho.mqtt.client as mqtt

def parse_line(line):
    # tolerant parser for HB and ANAL lines
    line = line.strip()
    if not line: 
        return None
    parts = line.split(',')
    if parts[0].upper() == 'HB':
        # example parts: HB,ts,DEVICE=0x0101,SEQ=125,BATT=84,RSSI=-72
        data = {'type':'HB'}
        try:
            data['ts'] = parts[1]
            for p in parts[2:]:
                if '=' in p:
                    k,v = p.split('=',1)
                    data[k.strip().lower()] = v.strip()
        except Exception as e:
            data['raw'] = line
        return data
    elif parts[0].upper() in ('ANAL','ANALYSIS'):
        data = {'type':'ANAL'}
        try:
            data['ts'] = parts[1]
            for p in parts[2:]:
                if '=' in p:
                    k,v = p.split('=',1)
                    data[k.strip().lower()] = v.strip()
        except Exception as e:
            data['raw'] = line
        return data
    else:
        # generic fallback
        return {'type':'RAW','raw':line, 'ts': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())}

def worker_serial_read(serial_port, csv_writer, csv_lock, mqtt_client, mqtt_topic):
    while True:
        try:
            line = serial_port.readline().decode(errors='ignore').strip()
            if not line:
                continue
            parsed = parse_line(line)
            # write to CSV
            with csv_lock:
                if parsed['type']=='HB':
                    csv_writer.writerow({
                        'ts': parsed.get('ts',''),
                        'type':'HB',
                        'device': parsed.get('device',''),
                        'seq': parsed.get('seq',''),
                        'batt': parsed.get('batt',''),
                        'rssi': parsed.get('rssi','')
                    })
                elif parsed['type']=='ANAL':
                    csv_writer.writerow({
                        'ts': parsed.get('ts',''),
                        'type':'ANAL',
                        'device': parsed.get('device',''),
                        'seq': parsed.get('seq',''),
                        'ph': parsed.get('ph',''),
                        'turb': parsed.get('turb',''),
                        'atp': parsed.get('atp',''),
                        'flags': parsed.get('flags','')
                    })
                else:
                    csv_writer.writerow({
                        'ts': parsed.get('ts',''),
                        'type': parsed.get('type','RAW'),
                        'raw': parsed.get('raw','')
                    })
                serial_port.flush()
            # publish to MQTT
            try:
                mqtt_client.publish(mqtt_topic, json.dumps(parsed))
            except Exception:
                pass
            print(line)
        except Exception as e:
            print("Serial read error:", e)
            time.sleep(0.5)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port','-p', default='/dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--csv', default='telemetry.csv')
    ap.add_argument('--mqtt', default='localhost')
    ap.add_argument('--topic', default='pipe/telemetry')
    args = ap.parse_args()

    # open serial
    ser = serial.Serial(args.port, args.baud, timeout=1)
    time.sleep(2)  # allow device reset

    # open csv for appending
    csvfile = open(args.csv, 'a', newline='')
    fieldnames = ['ts','type','device','seq','batt','rssi','ph','turb','atp','flags','raw']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    # write header if file empty
    try:
        csvfile.seek(0)
        if csvfile.read(1) == '':
            writer.writeheader()
    except Exception:
        pass

    csv_lock = threading.Lock()

    # MQTT client
    client = mqtt.Client()
    try:
        client.connect(args.mqtt, 1883, 60)
        client.loop_start()
    except Exception as e:
        print("MQTT connect failed:", e)

    print("Starting serial reader on", args.port)
    worker_serial_read(ser, writer, csv_lock, client, args.topic)

if __name__ == '__main__':
    main()

