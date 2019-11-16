#ifndef PID_H
#define PID_H

#include <stdint.h>

class PID{
public:
	// Initialize the PID class
	PID(const float kp, const float ki, const float kd, const uint32_t sample_time);

	// Set the limits for the generated output signal
	void setOutputLimits(const float min, const float max);

	// Set the PID tuning parameters
	void setTunings(const float kp, const float ki, const float kd);

	// Used by the class to resume AUTOMATIC PID function after MANUAL override
	void initialize();

	// Used to pause and resume PID, MANUAL or AUTOMATIC
	void setMode(const bool mode);

	// Computes the new output value
	void compute();

	// Set the PID variables
	void setInput(const float input);
	void setReference(const float reference);
	void setOutput(const float output);
	void setIntegral(const float integral);

	// Get the PID variables
	float getInput() const;
	float getReference() const;
	float getOutput() const;
	float getIntegral() const;

private:
	float _kp, _ki, _kd; // PID settings
	uint32_t _sample_time, _last_time;
	bool _mode;
	float _input, _output, _setpoint;
	float _integral, _last_input;
	float _out_max, _out_min;
};

#endif
