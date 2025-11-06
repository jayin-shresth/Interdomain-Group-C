#include <SPI.h>
#include <LoRa.h>

// ====== CONFIG ======
#define LORA_CS    5
#define LORA_RST   14
#define LORA_DIO0  26
#define LORA_FREQ  433E6

// ====== CRC ======
uint16_t crc16_ccitt(const uint8_t *buf, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= ((uint16_t)buf[i] << 8);
    for (uint8_t j = 0; j < 8; j++)
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
  }
  return crc;
}

// ====== PACKETS ======
struct Heartbeat {
  uint16_t id;
  uint32_t seq;
  uint32_t ts;
  uint8_t  phase;
  uint8_t  batt;
  int16_t  rssi;
  uint16_t crc;
} __attribute__((packed));

// ====== SETUP ======
void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa Base Station Init...");

  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init failed!");
    while (1);
  }
  LoRa.setSpreadingFactor(11);
  LoRa.setSignalBandwidth(125E3);
  LoRa.enableCrc();
  Serial.println("LoRa Base Ready!");
}

// ====== FUNCTIONS ======
void sendCommand(uint8_t cmd, uint8_t param) {
  uint8_t pkt[8];
  pkt[0] = cmd;
  pkt[1] = param;
  pkt[2] = random(0, 255);
  pkt[3] = random(0, 255);
  pkt[4] = random(0, 255);
  pkt[5] = random(0, 255);
  uint16_t crc = crc16_ccitt(pkt, 6);
  pkt[6] = (crc >> 8) & 0xFF;
  pkt[7] = crc & 0xFF;

  LoRa.beginPacket();
  LoRa.write(pkt, 8);
  LoRa.endPacket();
  Serial.println("Command sent!");
}

void receiveHeartbeat() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    uint8_t buf[64];
    int len = LoRa.readBytes(buf, packetSize);
    if (len >= sizeof(Heartbeat)) {
      Heartbeat hb;
      memcpy(&hb, buf, sizeof(Heartbeat));
      uint16_t calc_crc = crc16_ccitt(buf, sizeof(Heartbeat) - 2);
      if (calc_crc == hb.crc) {
        Serial.print("[HB] Seq: ");
        Serial.print(hb.seq);
        Serial.print(" Batt: ");
        Serial.print(hb.batt);
        Serial.print("% RSSI: ");
        Serial.println(LoRa.packetRssi());
      }
    }
  }
}

void loop() {
  receiveHeartbeat();

  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'b') {
      sendCommand(3, 5); // BURST_OPEN 5s
    } else if (ch == 'e') {
      sendCommand(1, 0); // E_STOP
    }
  }
}
