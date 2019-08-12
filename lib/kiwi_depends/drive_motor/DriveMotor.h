#ifndef DRIVEMOTOR_H
#define DRIVEMOTOR_H

#include <stdint.h>

class DriveMotor{
public:
	// Initialize motor
	DriveMotor(uint8_t in1, uint8_t in2);

	// Generate PWM control signal
	void set_pwm(int16_t pwm);

	// Get last PWM command
	int16_t get_pwm() const;

	// Sets frequency and resolution of the PWM
	// If different motors use the same timer, those values will be the same for all.
	void set_vals(float freq, uint8_t res);

	// Get PWM resolution to get command value range
	uint8_t get_res();
	float get_freq();

private:
	const uint8_t _in1, _in2;
	int16_t _pwm;

	// Default value in constructor is _freq = 23437.5 and _res = 11 as advised here :https://www.pjrc.com/teensy/td_pulse.html
	uint8_t _res;
	float _freq;
};

#endif
