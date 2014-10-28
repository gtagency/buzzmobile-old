#include <Arduino.h>
#include <Servo.h>
#include "pid.h"
#include <stdio.h>

#define STX ((char)2)

const int motor_pin = 3;
const int steer_pin = 5;
const int enc_a     = 2;
const int enc_b     = 4;
const int pot_pin   = A0;
const int led_pin   = 13;

Servo motor;
Servo steer;

volatile int ticks = 0;
volatile int count = 0;

const double ticksPerRev = 3600;
const double wheelCirc = 2.198;

const int minPotVal = 540;
const int maxPotVal = 775;
const int midPotVal = 660;
const float anglePerPotTick = 0.003697731; // rad

float desiredSpeed = 0; // m/s
float desiredAngle = 0; // rad

long lastTime;

PDController speedController(2, -0.1);
PDController steerController(15, -7.5);

long lastCmdTime;

float getSpeed() {
  float dTime = (millis() - lastTime) / 1000.0;
  lastTime = millis();
  float measured = 0;
  if(dTime > 0.001) {
    measured = ((ticks / ticksPerRev) * wheelCirc) / dTime;
    ticks = 0;
  }
  return measured;
}

float getSteeringAngle() {
  return -1 * anglePerPotTick * (analogRead(pot_pin) - midPotVal);
}

void stopAll() {
  speedController.setDesiredValue(0);
  speedController.setOutput(90);
  motor.write(90);
  steerController.setDesiredValue(getSteeringAngle());
  steerController.setOutput(90);
  steer.write(90);
}

void setup() {
  Serial.begin(9600);
  
  pinMode(motor_pin, OUTPUT);
  pinMode(steer_pin, OUTPUT);
  pinMode(enc_a, INPUT);
  pinMode(enc_b, INPUT);
  pinMode(led_pin, OUTPUT);
  
  lastTime = millis();
  
  attachInterrupt(0, onEncChange, CHANGE);
  
  motor.attach(motor_pin);
  steer.attach(steer_pin);
  
  speedController.setOutput(90);
  steerController.setOutput(90);
  speedController.setRange(0, 180);
  steerController.setRange(30, 150);
  speedController.setDesiredValue(0);
  steerController.setDesiredValue(0);
  
  digitalWrite(led_pin, LOW);
}

void loop() {
  char retMsg[6] = {0};
  while(Serial.available()) {
    if(Serial.read() == STX) {
      speedController.setDesiredValue(Serial.parseFloat());
      steerController.setDesiredValue(Serial.parseFloat());
      lastCmdTime = millis();
      retMsg[0] = STX;
      sprintf(retMsg+1, "%05i", count);
      Serial.println(retMsg);
      count = 0;
    }
  }
  
  speedController.update(getSpeed());
  motor.write(speedController.getOutput());
  
  
  steerController.update(getSteeringAngle());
  // Soft limits on steering to avoid damage.
  if(analogRead(pot_pin) >= maxPotVal && steerController.getOutput() < 90)
    steer.write(90);
  else if(analogRead(pot_pin) <= minPotVal && steerController.getOutput() > 90)
    steer.write(90);
  else
    steer.write(steerController.getOutput());
    
  
  if(millis() - lastCmdTime > 500) {
    digitalWrite(led_pin, HIGH);
    stopAll();
  } else {
    digitalWrite(led_pin, LOW);
  }
  
    
  delay(50);
}

void onEncChange() {
  if(digitalRead(enc_a) == digitalRead(enc_b)) {
    ticks--;
    count--;
  } else {
    ticks++;
    count++;
  }
}
