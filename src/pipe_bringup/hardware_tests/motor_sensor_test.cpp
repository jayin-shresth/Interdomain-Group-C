#include <Arduino.h>
#include <Wire.h>
#include <NewPing.h> 
#include <HardwareSerial.h>

#define MOTOR_PIN_1 16
#define MOTOR_PIN_2 17
#define MOTOR_PWM_PIN 5

// pins
#define ULTRASONIC_TRIG_PIN 12
#define ULTRASONIC_ECHO_PIN 13

HardwareSerial Serial2(2); 

Ultrasonic ultrasonic(ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN);

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17); 
  
  pinMode(MOTOR_PIN_1, OUTPUT);
  pinMode(MOTOR_PIN_2, OUTPUT);
  ledcSetup(0, 1000, 8);  
  ledcAttachPin(MOTOR_PWM_PIN, 0);

  Serial.println("ESP32 Motor and Ultrasonic test start");
}

void loop() {
  long distance = ultrasonic.read();
  
  
  Serial2.print("ultrasonic,");
  Serial2.println(distance);

  
  int motorSpeed = map(distance, 0, 100, 255, 0);  
  motorSpeed = constrain(motorSpeed, 0, 255);

  ledcWrite(0, motorSpeed);
  digitalWrite(MOTOR_PIN_1, HIGH);
  digitalWrite(MOTOR_PIN_2, LOW);
  
  delay(100);
}

