/*
 * PositionPIDTuner.h
 *
 *  Created on: Aug 4, 2026
 *      Author: nika-
 */

#ifndef INC_COMPONENT_POSITIONPIDTUNER_H_
#define INC_COMPONENT_POSITIONPIDTUNER_H_

#include <cmath>

#include "PositionPIDController.h"

enum PosTuningState{
	IDLE,
	MEASURING_SPEED_RESPONSE,
	CALCULATING,
	FINISHED
};

class PositionPIDTuner {
public:
	PositionPIDTuner(PositionPIDController* target_pid_);
	virtual ~PositionPIDTuner();

	void setPulse(float _pulse);

	void start(float settling_time = 0.2f, int16_t target_speed = 500);
	void update1ms();
	bool isFinished() const;

private:
	PositionPIDController* target_pid_;
	PosTuningState _state = PosTuningState::IDLE;

	float settling_time_ = 0.02f;
	int16_t target_speed_;
	uint32_t timer_ms_ = 0;
	float measured_Tv_ = 0.01f;

	float alpha = 0.2f;
	int32_t last_angle = 0; //速度計測用
	float filtered_speed = 0.0f;

	float t63_speed = 0.0f;

	float pulse = 8192.0f;
};

#endif /* INC_COMPONENT_POSITIONPIDTUNER_H_ */
