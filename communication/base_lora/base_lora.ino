/*
 base_lora_final.ino
 -------------------------------------
 Base Station for Pipe Rover Project
 - Receives LoRa packets (Heartbeat + LOG)
 - Prints standardized output over Serial for gateway
 - Allows typing commands (like "dump") to send LoRa commands to rover
 - Compatible with rover_lora_nosd.ino
*/

#include <SPI.h>
#include <LoRa.h>

// ---------- CONFIG ----------
#define LORA_CS    5
#define LORA_RST   14
#define LORA_DIO0  26
#define LORA_FREQ  433E6

// ---------- CRC16 ----------
uint16_t crc16_ccitt(const uint8_t *buf, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= ((uint16_t)buf[i] << 8);
    for (uint8_t j = 0; j < 8; j++)
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
  }
  return crc;
}

// ---------- HEARTBEAT STRUCT ----------
struct Heartbeat {
  uint16_t id;
  uint32_t seq;
  uint32_t ts;
  uint8_t  phase;
  uint8_t  batt;
  int16_t  rssi;
  uint16_t crc;
} __attribute__((packed));

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("💻 Base Station Booting...");
  Serial.println("Type 'dump' to request log dump from rover.");

  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("❌ LoRa init failed!");
    while (1);
  }

  LoRa.setSpreadingFactor(11);
  LoRa.setSignalBandwidth(125E3);
  Serial.println("✅ LoRa Ready");
}

// ---------- SEND ASCII COMMAND ----------
void sendAsciiCmd(const String &cmd) {
  LoRa.beginPacket();
  LoRa.print(cmd);
  LoRa.endPacket();
  Serial.print("📡 Sent LoRa Command: ");
  Serial.println(cmd);
}

// ---------- RECEIVE PACKETS ----------
void receivePacket() {
  int packetSize = LoRa.parsePacket();
  if (!packetSize) return;

  uint8_t buf[256];
  int len = LoRa.readBytes(buf, packetSize);

  // Try as ASCII first
  String s = "";
  for (int i = 0; i < len; i++) s += (char)buf[i];
  s.trim();

  // Handle LOG packets
  if (s.startsWith("LOG:")) {
    String after = s.substring(4); // chunkId:payload
    Serial.print("[LOG] ");
    Serial.println(after);
    return;
  }

  // Try to parse as Heartbeat
  if (len >= sizeof(Heartbeat)) {
    Heartbeat hb;
    memcpy(&hb, buf, sizeof(Heartbeat));
    uint16_t calc_crc = crc16_ccitt(buf, sizeof(Heartbeat) - 2);
    if (calc_crc == hb.crc && hb.id == 0xBEEF) {
      Serial.print("[HB] ");
      Serial.print((unsigned long)hb.seq);
      Serial.print(",");
      Serial.print((unsigned int)hb.batt);
      Serial.print(",");
      Serial.print(LoRa.packetRssi());
      Serial.print(",");
      Serial.println((unsigned long)hb.ts);
      return;
    }
  }

  // Fallback for any other text
  Serial.print("[RAW] ");
  Serial.println(s);
}

// ---------- LOOP ----------
String serialBuffer = "";

void loop() {
  // Listen for LoRa packets
  receivePacket();

  // Listen for user input (from laptop serial)
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      serialBuffer.trim();
      if (serialBuffer.length() > 0) {
        if (serialBuffer.equalsIgnoreCase("dump")) {
          sendAsciiCmd("CMD:DUMP");
        } else {
          sendAsciiCmd(serialBuffer);
        }
      }
      serialBuffer = "";
    } else {
      serialBuffer += c;
    }
  }
}

