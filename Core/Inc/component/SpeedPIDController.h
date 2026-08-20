/*
 * SpeedPIDController.h
 *
 *  Created on: Jul 31, 2026
 *      Author: nika-
 */

#ifndef INC_COMPONENT_SPEEDPIDCONTROLLER_H_
#define INC_COMPONENT_SPEEDPIDCONTROLLER_H_

#include <cmath>

#include <main.h>

#include "component/private/Actuator.h"
#include "component/private/Encoder.h"

struct PIDgain {
	float kp;
	float ki;
	float kd;
};

class SpeedPIDController {
public:
	SpeedPIDController(Actuator* _act, Encoder* _enc);
	virtual ~SpeedPIDController();

	void setTarget(int16_t _target);

	void setPID(float _kp, float _ki, float _kd);
	void setPID(PIDgain _gain);
	void setAllowError(int16_t _allowError);
	void setMaxIntegral(float _max_integral);
	void setMaxOutput(int16_t _max_output);
	void setPulse(float _pulse);
	int16_t getMaxOutput(){ return max_output; };
	void setInterval(float _dt);
	void update();
	void reset();

	void enable();
	void disable();

	void lock();
	void unlock();

	Actuator* act;
	Encoder* enc;

private:
	int32_t target = 0;

	PIDgain gain = {
			.kp = 0.0f,
			.ki = 0.0f,
			.kd = 0.0f
	};
	float max_integral = 10.0f;
	float integral = 0.0f;
	float last_error = 0.0f;
	float dt = 0.01f;
	int16_t allowError = 10; //許容誤差
	int16_t max_output = 4096;
	int32_t last_angle = 0;
	float pulse = 8192.0f;

	bool locked = false;
	bool effective = true;
};

#endif /* INC_COMPONENT_SPEEDPIDCONTROLLER_H_ */
