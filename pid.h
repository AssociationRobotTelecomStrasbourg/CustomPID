#ifndef PID_H
#define PID_H

#include <stdint.h>

const uint8_t AUTOMATIC = 1;
const uint8_t MANUAL = 0;
const uint8_t DIRECT = 1;
const uint8_t REVERSE = 0;

class PID{
public:
	//PID constructor, initializes main variables.
	PID(const float kp, const float ki, const float kd, const uint32_t sampleTime);

	//Used to pause and resume PID, MANUAL or AUTOMATIC
	void setMode(const uint8_t mode);

	//Computes the next output value, needs to first update the input with setInput, and then get new value with getOutput
	//This implementation prevent side-effects and makes implementation in libraries easier.
	bool compute(const uint32_t time);
	inline void setInput(const float input) {_input = input;};
	inline void setReference(const float reference) {_setpoint = reference;};
	inline float getOutput() {return _output;};

	//Sets the limits for the generated output signal
	void setOutputLimits(const float min, const float max);

	//Sets the PID parameters
	void setTunings(const float kp, const float ki, const float kd);

	//Reverse the PID direction
	void reverse();

	//Functions for display purposes
	inline float kp() {return _kp;};
	inline float ki() {return _ki;};
	inline float kd() {return _kd;};
	inline float sampleTime() {return _sample_time;};

private:
	//Used by the class to resume AUTOMATIC PID function after MANUAL override
	void initialize();

	float _kp, _ki, _kd;
	float _input, _output, _setpoint;
	float _integral, _last_input;
	float _out_max, _out_min;

	uint32_t _sample_time, _last_time;
	uint8_t _mode, _direction;
};

#endif
