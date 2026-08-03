/*
 * PositionPIDController.h
 *
 *  Created on: Jul 31, 2026
 *      Author: nika-
 */

#ifndef INC_COMPONENT_POSITIONPIDCONTROLLER_H_
#define INC_COMPONENT_POSITIONPIDCONTROLLER_H_

#include <cmath>

#include "main.h"
#include "SpeedPIDController.h"

class PositionPIDController {
public:
	PositionPIDController(SpeedPIDController* speed_pid_);
	virtual ~PositionPIDController();

	void setTarget(int32_t _target);

	void setPID(float _kp);
	void setAllowError(int16_t _allowError);
	void setMaxSpeed(int16_t _max_speed);
	void setInterval(float _dt);
	void setMaxAcceleration(int16_t accele);
	void update();

	void reset();

	void enable();
	void disable();

	void lock();
	void unlock();

	SpeedPIDController* speed_pid_;
private:

	int32_t target;
	int16_t target_speed; //現在の出力
	int16_t max_acceleration = 100; //最大加速

	PIDgain gain = {
			.kp = 0.0f,
			.ki = 0.0f,
			.kd = 0.0f
	};
	int16_t allowError; //許容誤差
	int16_t max_speed;
	float dt = 0.01f;

	bool locked;
	bool startFlag;
	bool effective;

	int16_t out;
};

#endif /* INC_COMPONENT_POSITIONPIDCONTROLLER_H_ */
