#include "pid.h"

PID::PID(const float kp, const float ki, const float kd) : _mode(false), _anti_windup(true), _input(0), _output(0), _setpoint(0) {
	this->setOutputLimits(-255, 255);
	this->setTunings(kp, ki, kd);
}

void PID::setMode(const bool mode) {
	// Initialize if mode is enabled
	if (mode && !_mode)
		this->initialize();

	_mode = mode;
}

void PID::setOutputLimits(float min, float max) {
	_out_min = min;
	_out_max = max;

	if (_output > _out_max) {
		_output = _out_max;
	}
	else if (_output < _out_min) {
		_output = _out_min;
	}
}

void PID::setAntiWindup(const bool anti_windup) {
	_anti_windup = anti_windup;
}

void PID::setTunings(const float kp, const float ki, const float kd) {
	_kp = kp;
	_ki = ki;
	_kd = kd;
}

void PID::initialize() {
	_integral = _output;
	_last_input = _input;
}

void PID::compute() {
	if (_mode){
		// Compute the error and working variables:
		float error = _setpoint - _input;
		_integral += (_ki * error);
		float d_input = (_input - _last_input);

		// Compute PID output
		_output = _kp * error + _integral - _kd * d_input;

		// Saturate output and apply anti-windup if activated
		if (_output > _out_max){
			_output = _out_max;
			if (_anti_windup)
				_integral -= (_ki * error); // Reset integral term
		}
		else if (_output < _out_min){
			_output = _out_min;
			if (_anti_windup)
				_integral -= (_ki * error);
		}

		// Remember some variables for next time
		_last_input = _input;
	}
}

void PID::setInput(const float input) {
	_input = input;
}

void PID::setSetpoint(const float reference) {
	_setpoint = reference;
}

void PID::setOutput(const float output) {
	_output = output;
}

void PID::setIntegral(const float integral) {
	_integral = integral;
}

float PID::getInput() const {
	return _input;
}

float PID::getSetpoint() const {
	return _setpoint;
}

float PID::getOutput() const {
	return _output;
}

float PID::getIntegral() const {
	return _integral;
}
