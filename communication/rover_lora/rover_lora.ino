/*
 rover_lora_nosd.ino
 -------------------------------------
 Rover node :
 - Sends periodic Heartbeats (binary struct with CRC16)
 - Stores mission breadcrumbs in memory (RAM)
 - On command "CMD:DUMP" from base, transmits all logs as "LOG:<chunkId>:<data>"
*/

#include <SPI.h>
#include <LoRa.h>

// ---------- CONFIG ----------
#define LORA_CS    5
#define LORA_RST   14
#define LORA_DIO0  26
#define LORA_FREQ  433E6

const int PAYLOAD_CHUNK = 150; // safe payload size per LoRa packet

// ---------- CRC16-CCITT ----------
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
  uint16_t id;     // constant identifier
  uint32_t seq;
  uint32_t ts;
  uint8_t  phase;
  uint8_t  batt;
  int16_t  rssi;   // unused on TX
  uint16_t crc;
} __attribute__((packed));

// ---------- GLOBALS ----------
uint32_t seqCounter = 0;
unsigned long lastHB = 0;
const unsigned long HB_INTERVAL_MS = 3000;
bool dumping = false;

// ---------- In-Memory Mission Logs ----------
#define MAX_LOGS 300  
String missionLogs[MAX_LOGS];
int logCount = 0;

// ---------- Logging ----------
void logBreadcrumb(uint32_t seq, const String &phase,
                   float x, float y, float heading,
                   float ph, float turb, uint8_t batt) {
  if (logCount >= MAX_LOGS) return; // Prevent overflow

  char line[150];
  snprintf(line, sizeof(line),
           "%lu,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%u,%lu",
           (unsigned long)seq, phase.c_str(),
           x, y, heading, ph, turb,
           (unsigned int)batt, (unsigned long)millis());

  missionLogs[logCount++] = String(line);
  Serial.printf("[LOGGED] #%d %s\n", logCount, line);
}

// ---------- Transmit All Logs ----------
void transmitLogs() {
  if (dumping) {
    Serial.println("Already dumping, skip");
    return;
  }

  if (logCount == 0) {
    Serial.println("No logs to transmit");
    return;
  }

  dumping = true;
  Serial.printf(" Transmitting %d log lines...\n", logCount);

  for (int i = 0; i < logCount; i++) {
    LoRa.beginPacket();
    LoRa.print("LOG:");
    LoRa.print(i);
    LoRa.print(":");
    LoRa.print(missionLogs[i]);
    LoRa.endPacket();
    delay(250);  // pace transmission
  }

  Serial.println("📤 Log transmission complete");
  dumping = false;
}

// ---------- Send Heartbeat ----------
void sendHeartbeat() {
  Heartbeat hb;
  hb.id = 0xBEEF;
  hb.seq = seqCounter++;
  hb.ts = millis();
  hb.phase = 1;
  hb.batt = random(70, 100);
  hb.rssi = 0;
  hb.crc = crc16_ccitt((uint8_t*)&hb, sizeof(Heartbeat) - 2);

  LoRa.beginPacket();
  LoRa.write((uint8_t*)&hb, sizeof(Heartbeat));
  LoRa.endPacket();

  Serial.printf("[HB] Sent seq=%lu, CRC=%04X\n", hb.seq, hb.crc);
}

// ---------- Handle Commands ----------
void parseIncomingCommands() {
  int packetSize = LoRa.parsePacket();
  if (!packetSize) return;

  String incoming = "";
  while (LoRa.available()) incoming += (char)LoRa.read();
  incoming.trim();

  Serial.print("[CMD] Received: ");
  Serial.println(incoming);

  if (incoming == "CMD:DUMP") {
    Serial.println("CMD:DUMP received -> transmitting logs");
    transmitLogs();
  } else {
    Serial.println("Unknown command");
  }
}

// ---------- Placeholder: Detect Return to Base ----------
bool isAtOrigin() {
  #include <WiFi.h>

bool isAtOrigin() {
  int n = WiFi.scanNetworks(false, true);
  for (int i = 0; i < n; i++) {
    String ssid = WiFi.SSID(i);
    int rssi = WiFi.RSSI(i);
    if (ssid == "BASE_Rover" && rssi > -40) {
      Serial.printf("Detected Base Wi-Fi (%s) RSSI=%d dBm\n", ssid.c_str(), rssi);
      return true;  // close to origin
    }
  }
  return false;
}

  return false;
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println(" Rover (No SD) Booting...");

  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println(" LoRa init failed!");
    while (1);
  }

  LoRa.setSpreadingFactor(11);
  LoRa.setSignalBandwidth(125E3);
  Serial.println(" LoRa Ready");
  lastHB = millis();
}

// ---------- LOOP ----------
void loop() {
  unsigned long now = millis();

  // Send heartbeat every interval
  if (now - lastHB >= HB_INTERVAL_MS) {
    sendHeartbeat();
    lastHB = now;

    // Log a new breadcrumb entry
    logBreadcrumb(seqCounter, String("navigate"),
                  random(0,100)/10.0, random(0,100)/10.0,
                  random(0,360),
                  random(680,740)/100.0, random(40,90),
                  random(70,100));
  }

  // Check for LoRa commands
  parseIncomingCommands();

  //  auto transmit when at origin
  static bool dumpedOnce = false;
  if (!dumpedOnce && isAtOrigin()) {
    Serial.println("At origin detected -> sending logs");
    transmitLogs();
    dumpedOnce = true;
  }

  delay(10);
}


