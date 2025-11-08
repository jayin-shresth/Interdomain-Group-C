#include <SPI.h>
#include <LoRa.h>

// ===== CONFIG =====
#define LORA_CS   5
#define LORA_RST 14
#define LORA_DIO0 26
#define LORA_FREQ 433E6

// ===== CRC16-CCITT =====
uint16_t crc16_ccitt(const uint8_t *buf, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= ((uint16_t)buf[i] << 8);
    for (uint8_t j = 0; j < 8; j++)
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
  }
  return crc;
}

// ===== PACKET STRUCT =====
struct Heartbeat {
  uint16_t id;     // 0xBEEF
  uint32_t seq;
  uint32_t ts;
  uint8_t  phase;
  uint8_t  batt;
  int16_t  rssi;   // unused on TX
  uint16_t crc;
} __attribute__((packed));

uint32_t seq = 0;

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("🛰️ Rover LoRa Node Init...");

  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init failed!");
    while (1);
  }
  LoRa.setSpreadingFactor(11);
  LoRa.setSignalBandwidth(125E3);
  LoRa.enableCrc();
  Serial.println("✅ Rover LoRa Ready!");
}

// ===== MAIN LOOP =====
void loop() {
  Heartbeat hb;
  hb.id   = 0xBEEF;
  hb.seq  = seq++;
  hb.ts   = millis();
  hb.phase = 1; // navigate
  hb.batt  = random(70, 100);
  hb.rssi  = 0;

  hb.crc = crc16_ccitt((uint8_t*)&hb, sizeof(Heartbeat) - 2);

  LoRa.beginPacket();
  LoRa.write((uint8_t*)&hb, sizeof(Heartbeat));
  LoRa.endPacket();

  Serial.printf("Sent HB Seq %lu | CRC %04X\n", hb.seq, hb.crc);

  delay(3000);
}

