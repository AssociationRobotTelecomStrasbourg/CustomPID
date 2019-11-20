#ifndef PID_H
#define PID_H

#include <stdint.h>

class PID{
public:
	// Initialize the PID class
	PID(const float kp, const float ki, const float kd);

	// Enable or disable the PID
	void setMode(const bool mode);

	// Set the limits for the generated output signal
	void setOutputLimits(const float min, const float max);

	// Enable or disable the anti-windup
	void setAntiWindup(const bool anti_windup);

	// Set the PID tuning parameters
	void setTunings(const float kp, const float ki, const float kd);

	// Initialize variables of the PID when it is activated
	void initialize();

	// Computes the new output value
	void compute();

	// Set the PID variables
	void setInput(const float input);
	void setSetpoint(const float reference);
	void setOutput(const float output);
	void setIntegral(const float integral);

	// Get the PID variables
	float getInput() const;
	float getSetpoint() const;
	float getOutput() const;
	float getIntegral() const;

private:
	bool _mode; // Enable or disable the PID
	float _out_max, _out_min; // Limits of the output
	bool _anti_windup; // Enable or disable the anti-windup
	float _kp, _ki, _kd; // PID settings
	float _input, _output, _setpoint; // Variables of the PID
	float _integral; // Integral term
	float _last_input; // Last input to calculate derivative term
};

#endif
