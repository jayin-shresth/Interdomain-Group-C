// ESP32 Motor test with BTS7960 driver

#include <Arduino.h>

#define MOTOR_PIN_ENA 5  // PWM pin for motor speed (ENA)
#define MOTOR_PIN_IN1 18 // Direction pin 1 (IN1)
#define MOTOR_PIN_IN2 19 // Direction pin 2 (IN2)

void setup() {
  Serial.begin(115200);
  
  pinMode(MOTOR_PIN_ENA, OUTPUT);
  pinMode(MOTOR_PIN_IN1, OUTPUT);
  pinMode(MOTOR_PIN_IN2, OUTPUT);

  Serial.println("BTS7960 single motor test start");
}

void loop() {
  // Rotate forward at half speed
  digitalWrite(MOTOR_PIN_IN1, HIGH);
  digitalWrite(MOTOR_PIN_IN2, LOW);
  ledcSetup(0, 20000, 8);  // Use PWM channel 0, 20 kHz, 8-bit resolution
  ledcAttachPin(MOTOR_PIN_ENA, 0);
  ledcWrite(0, 128);

  Serial.println("Motor running forward at half speed");
  delay(3000);

  // Stop motor
  ledcWrite(0, 0);
  Serial.println("Motor stopped");
  delay(2000);

  // Rotate backward at full speed
  digitalWrite(MOTOR_PIN_IN1, LOW);
  digitalWrite(MOTOR_PIN_IN2, HIGH);
  ledcWrite(0, 255);

  Serial.println("Motor running backward at full speed");
  delay(3000);

  // Stop motor
  ledcWrite(0, 0);
  Serial.println("Motor stopped");
  delay(2000);
}

