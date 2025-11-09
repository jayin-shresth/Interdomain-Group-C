#include <Arduino.h>

#define TRIG_PIN_FL 12  // Front Left ultrasonic TRIG
#define ECHO_PIN_FL 13  // Front Left ultrasonic ECHO

#define TRIG_PIN_FR 14  // Front Right ultrasonic TRIG
#define ECHO_PIN_FR 27  // Front Right ultrasonic ECHO

#define TRIG_PIN_L  33  // Left ultrasonic TRIG
#define ECHO_PIN_L  32  // Left ultrasonic ECHO

#define TRIG_PIN_R  25  // Right ultrasonic TRIG
#define ECHO_PIN_R  26  // Right ultrasonic ECHO

long readUltrasonicDistance(int triggerPin, int echoPin) {
  long duration, distance;
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  duration = pulseIn(echoPin, HIGH, 30000); // 30 ms timeout
  if (duration == 0) {
    distance = -1; // no echo
  } else {
    distance = duration * 0.0343 / 2;
  }
  return distance;
}

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN_FL, OUTPUT);
  pinMode(ECHO_PIN_FL, INPUT);

  pinMode(TRIG_PIN_FR, OUTPUT);
  pinMode(ECHO_PIN_FR, INPUT);

  pinMode(TRIG_PIN_L, OUTPUT);
  pinMode(ECHO_PIN_L, INPUT);

  pinMode(TRIG_PIN_R, OUTPUT);
  pinMode(ECHO_PIN_R, INPUT);

  Serial.println("Starting 4 HC-SR04 sensor test");
}

void loop() {
  long distFL = readUltrasonicDistance(TRIG_PIN_FL, ECHO_PIN_FL);
  long distFR = readUltrasonicDistance(TRIG_PIN_FR, ECHO_PIN_FR);
  long distL = readUltrasonicDistance(TRIG_PIN_L, ECHO_PIN_L);
  long distR = readUltrasonicDistance(TRIG_PIN_R, ECHO_PIN_R);

  Serial.print("Front Left: ");
  Serial.print(distFL);
  Serial.print(" cm, Front Right: ");
  Serial.print(distFR);
  Serial.print(" cm, Left: ");
  Serial.print(distL);
  Serial.print(" cm, Right: ");
  Serial.print(distR);
  Serial.println(" cm");

  delay(500);
}

