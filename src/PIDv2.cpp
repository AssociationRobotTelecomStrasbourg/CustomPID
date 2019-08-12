#include "PIDv2.h"

PID::PID(const float kp, const float ki, const float kd, const uint32_t sampleTime)
	: _kp(kp), _ki(ki), _kd(kd), _sample_time(sampleTime), _mode(MANUAL){

}

void PID::setMode(const uint8_t mode){
	uint8_t new_mode = mode;
	if(new_mode != _mode){
		PID::initialize();
	}
	_mode = new_mode;
}

bool PID::compute(){
	if (_mode == MANUAL) return 0;

	uint32_t now = millis();
	uint32_t time_change = (now - _last_time);

	if (time_change >= _sample_time){
		//Compute the error and working variables:
		float error = _setpoint - _input;
		_integral += (_ki * error);
		float d_input = (_input - _last_input);

		//Compute PID output
		float output = _kp * error + _integral - _kd * d_input;

		//clamping integral and output (anti-windup)
		if (output > _out_max){
			_integral -= output - _out_max;
			output = _out_max;
		}
		else if (output < _out_min){
			_integral += _out_min - output;
			output = _out_min;
		}

		_output = output;

		//Remember some variables for next time
		_last_input = _input;
		_last_time = now;

		return true;
	}
	else{
		return false;
	}
}

void PID::initialize(){
	_integral = _output;
	_last_input = _input;
	if (_integral > _out_max) _integral = _out_max;
	else if (_integral < _out_min) _integral = _out_min;
}

void PID::setOutputLimits(float min, float max){
	if(min >= max) return;
  	_out_min = min;
	_out_max = max;

  	if(_mode)
  	{
	  //Clamp output and integral
  	}
}
