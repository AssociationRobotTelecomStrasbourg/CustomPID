#include "pid.h"

PID::PID(const float kp, const float ki, const float kd, const uint32_t sampleTime) :
_sample_time(sampleTime), _mode(true),
_input(0), _output(0), _setpoint(0) {
	this->setOutputLimits(-255, 255);
	this->setTunings(kp, ki, kd);
	this->initialize();
}

void PID::setMode(const bool mode){
	// Initialize if the we enable
	if (mode && !_mode)
		this->initialize();

	_mode = mode;
}

void PID::compute(const uint32_t time) {
	if (_mode && time - _last_time >= _sample_time){
		//Compute the error and working variables:
		float error = _setpoint - _input;
		_integral += (_ki * error);
		float d_input = (_input - _last_input);

		//Compute PID output
		float output = _kp * error + _integral - _kd * d_input;

		//clamping integral and output (anti-windup)
		if (output > _out_max){
			_integral -= (_ki * error);
			output = _out_max;
		}
		else if (output < _out_min){
			_integral -= (_ki * error);
			output = _out_min;
		}

		_output = output;

		//Remember some variables for next time
		_last_input = _input;
		_last_time = time;
	}
}

void PID::initialize(){
	_integral = _output;
	_last_input = _input;

	if (_integral > _out_max)
		_integral = _out_max;
	else if (_integral < _out_min)
		_integral = _out_min;
}

void PID::setOutputLimits(float min, float max){
	if (min >= max) return;

  	_out_min = min;
	_out_max = max;

  	if (_mode)
  	{
		if (_output > _out_max){
  		  _integral -= _output - _out_max;
  		  _output = _out_max;
  	  }
  	  else if (_output < _out_min){
  		  _integral += _out_min - _output;
  		  _output = _out_min;
  	  }
  	}
}

void PID::setTunings(const float kp, const float ki, const float kd){
	_kp = kp;
	_ki = ki * _sample_time;
	_kd = kd / _sample_time;
}
