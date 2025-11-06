#include <SPI.h>
#include <LoRa.h>

// ====== CONFIG ======
#define DEVICE_ID      0x0101
#define LORA_CS        5      // Chip Select
#define LORA_RST       14     // Reset
#define LORA_DIO0      26     // DIO0
#define LORA_FREQ      433E6  // Hz
#define SF_ROBUST      11
#define SF_BURST       7
#define BW_125K        125E3

// ====== VARIABLES ======
uint32_t seq_no = 0;
bool burstMode = false;
unsigned long burstDeadline = 0;
unsigned long lastHeartbeat = 0;
unsigned long hbInterval = 3000; // 3 sec

// ====== CRC16 ======
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

  Serial.println("LoRa Robot Node Init...");

  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init failed!");
    while (1);
  }

  LoRa.setSpreadingFactor(SF_ROBUST);
  LoRa.setSignalBandwidth(BW_125K);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();
  LoRa.setTxPower(17);

  Serial.println("LoRa Robot Ready!");
}

// ====== FUNCTIONS ======
void sendHeartbeat() {
  Heartbeat hb;
  hb.id = DEVICE_ID;
  hb.seq = seq_no++;
  hb.ts = millis() / 1000;
  hb.phase = 1;      // navigating
  hb.batt = 85;      // dummy battery %
  hb.rssi = 0;
  uint8_t *ptr = (uint8_t*)&hb;
  hb.crc = crc16_ccitt(ptr, sizeof(Heartbeat) - 2);

  LoRa.beginPacket();
  LoRa.write((uint8_t*)&hb, sizeof(Heartbeat));
  LoRa.endPacket();

  Serial.print("Sent HB seq: ");
  Serial.println(hb.seq);
}

// Handle command packets from base
void handleCommand(uint8_t *buf, int len) {
  uint8_t cmd = buf[0];
  uint8_t param = buf[1];
  if (cmd == 3) {  // BURST_OPEN
    burstMode = true;
    burstDeadline = millis() + param * 1000UL;
    LoRa.setSpreadingFactor(SF_BURST);
    hbInterval = 800;
    Serial.println("BURST mode started");
  } else if (cmd == 1) { // E_STOP
    Serial.println("!!! E-STOP Received !!!");
    // stop robot logic here
  }
}

void checkIncoming() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    uint8_t buf[64];
    int len = LoRa.readBytes(buf, packetSize);
    uint16_t rx_crc = (buf[len - 2] << 8) | buf[len - 1];
    if (crc16_ccitt(buf, len - 2) == rx_crc) {
      handleCommand(buf, len);
    }
  }
}

void loop() {
  checkIncoming();

  if (burstMode && millis() > burstDeadline) {
    burstMode = false;
    LoRa.setSpreadingFactor(SF_ROBUST);
    hbInterval = 3000;
    Serial.println("Back to normal mode");
  }

  if (millis() - lastHeartbeat > hbInterval) {
    sendHeartbeat();
    lastHeartbeat = millis();
  }
}
