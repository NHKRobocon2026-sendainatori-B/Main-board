/*
 * PID.h
 *
 *  Created on: Jul 4, 2026
 *      Author: nika-
 */

#ifndef INC_COMPONENT_PID_H_
#define INC_COMPONENT_PID_H_

#include <main.h>

#include "component/private/Actuator.h"
#include "component/private/Encoder.h"

class PID {
public:
	PID(Actuator* _act, Encoder* _enc);
	virtual ~PID();

	void setTarget(int32_t _target);

	void setPID(float _kp, float _ki, float _kd);
	void setAllowError(int16_t _allowError);
	void setMaxIntegral(float _max_integral);
	void setMaxOutput(int16_t _max_output);
	void setInterval(uint8_t _dt);
	void calculatePID();
	void reset();

	void enable();
	void disable();

	void lock();
	void unlock();

private:
	Actuator* act;
	Encoder* enc;

	int32_t target;

	float kp, ki, kd;
	float max_integral;
	float integral;
	float last_error;
	float dt;
	int16_t allowError; //許容誤差
	int16_t max_output;

	bool locked;
	bool startFlag;
	bool effective;
};

#endif /* INC_COMPONENT_PID_H_ */
