#ifndef PIDV2_H
#define PIDV2_H

#include <stdint.h>
#include <Arduino.h>

const uint8_t AUTOMATIC {1};
const uint8_t MANUAL {0};

class PID{

public:

	//PID constructor, initializes main variables.
	PID(const float kp, const float ki, const float kd, const uint32_t sampleTime);

	//Used to pause and resume PID, MANUAL or AUTOMATIC
	void setMode(const uint8_t mode);

	//Computes the next output value, needs to first update the input with setInput, and then get new value with getOutput
	bool compute();
	void setInput(const float input);
	void setReference(const float reference);
	float getOutput();

	//Sets the limits for the generated output signal
	void setOutputLimits(const float min, const float max);

protected:

	//Used by the class to resume AUTOMATIC PID function after MANUAL override
	void initialize();

private:
	float _kp, _ki, _kd;
	float _input, _output, _setpoint;
	float _integral, _last_input;
	float _out_max, _out_min;

	uint32_t _sample_time, _last_time;
	uint8_t _mode;
};

#endif
