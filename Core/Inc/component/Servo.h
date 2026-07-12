/*
 * Servo.h
 *
 *  Created on: Jul 11, 2026
 *      Author: nika-
 */

#ifndef INC_COMPONENT_SERVO_H_
#define INC_COMPONENT_SERVO_H_

#include "main.h"

class Servo {
public:
	Servo(TIM_HandleTypeDef* _tim_handle, uint16_t _tim_channel);
	virtual ~Servo();

	void move(uint16_t angle);
	void setting(uint16_t _out0, uint16_t _out180);

	void lock();
	void unlock();

private:
	TIM_HandleTypeDef* tim_handle;
	uint16_t tim_channel;
	uint16_t out0; //0度の時の出力
	uint16_t out180; //180度の出力

	bool locked;
};

#endif /* INC_COMPONENT_SERVO_H_ */
