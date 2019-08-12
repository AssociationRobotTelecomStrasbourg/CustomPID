#include <Arduino.h>
#include <Encoder.h>
#include "PIDv2.h"

#include "drive_motor/DriveMotor.h"
#include "miniKiwiPins.h"

uint8_t res = 0;
const uint16_t max_pwm = 0;

const uint32_t sample_time = 5;
const uint32_t display_time = 100;
uint32_t time;

DriveMotor motor1(IN4, IN3);
Encoder enc(ENC_A1, ENC_B1);

PID myPID(6, 0, 0, sample_time);

void setup() {
	delay(5000);
    Serial.begin(9600);
    Serial.println("Starting PID Test");
    pinMode(DEBUG, OUTPUT);
    digitalWrite(DEBUG, HIGH);

	res = motor1.get_res();
	float max_pwm = 2<<(res-1);
	Serial.print("Max PWM value calculated : +-");
	Serial.println(max_pwm);

	myPID.setOutputLimits(-max_pwm, max_pwm);

	myPID.setInput(enc.read());
	myPID.setReference(1200);

	myPID.setMode(AUTOMATIC);

	time = millis();
}

void loop() {
  myPID.setInput(enc.read());
  myPID.compute();
  motor1.set_pwm(int16_t(myPID.getOutput()));
  if (millis() - time > display_time) {
	  time += display_time;
	  Serial.print(enc.read());
	  Serial.print(" ");
	  Serial.println(myPID.getOutput());
  }
}
