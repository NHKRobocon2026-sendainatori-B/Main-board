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

	void setPID(float _kp, float _ki,float _kd);
	void setPID(PIDgain _gain);
	void setAllowError(int16_t _allowError);
	void setMaxIntegral(float _max_integral);
	void setMaxSpeed(int16_t _max_speed);
	void setInterval(float _dt);
	void setMaxAcceleration(int32_t accele);
	void update();

	void reset();

	void enable();
	void disable();

	void lock();
	void unlock();

	bool isTargetReached() const {
		return (std::fabs(last_error) <= allowError);
	}

	SpeedPIDController* speed_pid_;
private:

	int32_t target = 0;
	int16_t target_speed = 0; //現在の出力
	int32_t max_acceleration = 100; //最大加速

	PIDgain gain = {
			.kp = 0.0f,
			.ki = 0.0f,
			.kd = 0.0f
	};
	float max_integral = 1000.0f;
	float integral = 0.0f;
	int16_t allowError = 10; //許容誤差
	int16_t max_speed = 4096;
	int32_t last_error = 0;
	float dt = 0.01f;

	bool locked = false;
	bool effective = true;
};

#endif /* INC_COMPONENT_POSITIONPIDCONTROLLER_H_ */
