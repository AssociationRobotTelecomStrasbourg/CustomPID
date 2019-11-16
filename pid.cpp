#include "pid.h"

PID::PID(const float kp, const float ki, const float kd, const uint32_t sampleTime) :
_sample_time(sampleTime), _mode(false), _input(0), _output(0), _setpoint(0) {
    this->setOutputLimits(-255, 255);
    this->setTunings(kp, ki, kd);
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

void PID::setTunings(const float kp, const float ki, const float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PID::initialize() {
    _integral = _output;
    _last_input = _input;

    if (_integral > _out_max)
        _integral = _out_max;
    else if (_integral < _out_min)
        _integral = _out_min;
}

void PID::setMode(const bool mode) {
    // Initialize if mode is enabled
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
        _output = _kp * error + _integral - _kd * d_input;

        //clamping integral and output (anti-windup)
        if (_output > _out_max){
            _integral -= (_ki * error);
            _output = _out_max;
        }
        else if (_output < _out_min){
            _integral -= (_ki * error);
            _output = _out_min;
        }

        //Remember some variables for next time
        _last_input = _input;
        _last_time += _sample_time;
    }
}

void PID::setInput(const float input) {
    _input = input;
}

void PID::setReference(const float reference) {
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

float PID::getReference() const {
    return _setpoint;
}

float PID::getOutput() const {
    return _output;
}

float PID::getIntegral() const {
    return _integral;
}
