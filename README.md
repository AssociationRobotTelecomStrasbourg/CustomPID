# CustomPID
Simple Arduino PID library based on Brett's tutorials with improved style and interface.

```c++
#include <Encoder.h>
#include "PIDv2.h"

#include "drive_motor/DriveMotor.h"
#include "miniKiwiPins.h"

//Example code, using the library to do position control of a DC motor

uint8_t res = 0;
const uint16_t max_pwm = 0;

const uint32_t sample_time = 5;
const uint32_t display_time = 100;
uint32_t time;

DriveMotor motor1(IN4, IN3);
Encoder enc(ENC_A1, ENC_B1);

//Initializing the PID object
PID myPID(16, 0.6, 1.1, sample_time);

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

	//After calculating the pwm values using the set pwm resolution, we set the limits of the PWM output
	myPID.setOutputLimits(-max_pwm, max_pwm);

	//Then, initialize the input by reading the encoder, and setting the desired value to 2400 (2 turns)
	myPID.setInput(enc.read());
	myPID.setReference(2400.0);

	//Setting mode to AUTOMATIC turns on the PID
	myPID.setMode(AUTOMATIC);

	time = millis();
}

void loop() {
	//Main loop needs to run these computation, using get/set for the input/outputs of the PID and calling the compute() function
	myPID.setInput(enc.read());
	myPID.compute();
	motor1.set_pwm(int16_t(myPID.getOutput()));

	//Code used to display the values at a different refresh rate of the PID
	if (millis() - time > display_time) {
		time += display_time;
		Serial.print(enc.read());
		Serial.print(" ");
		Serial.println(myPID.getOutput());
  }
}
```
