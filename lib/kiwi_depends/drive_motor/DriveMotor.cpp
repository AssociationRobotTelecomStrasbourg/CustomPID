#include "DriveMotor.h"
#include <Arduino.h>

DriveMotor::DriveMotor(uint8_t in1, uint8_t in2) : _in1(in1), _in2(in2), _res(11), _freq(23437.5) {
	pinMode(_in1, OUTPUT);
	pinMode(_in2, OUTPUT);

	analogWrite(_in1, 0);
	analogWrite(_in2, 0);

	analogWriteResolution(_res);
	analogWriteFrequency(_in1, _freq);
	analogWriteFrequency(_in2, _freq);
}

void DriveMotor::set_pwm(int16_t pwm){
	if (pwm > 0){
		analogWrite(_in1, pwm);
		analogWrite(_in2, 0);
	}
	else{
		analogWrite(_in1, 0);
		analogWrite(_in2, -pwm);
	}
	_pwm = pwm;
}

int16_t DriveMotor::get_pwm() const{
	return(_pwm);
}

void DriveMotor::set_vals(float freq, uint8_t res){
	_freq = freq;
	_res = res;
	analogWriteResolution(_res);
	analogWriteFrequency(_in1, _freq);
	analogWriteFrequency(_in2, _freq);
}

uint8_t DriveMotor::get_res(){
	return(_res);
}

float DriveMotor::get_freq(){
	return(_freq);
}
