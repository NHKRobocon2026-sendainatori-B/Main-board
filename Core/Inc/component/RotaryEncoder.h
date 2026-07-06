/*
 * RotaryEncoder.h
 *
 *  Created on: Jul 4, 2026
 *      Author: nika-
 */

#ifndef INC_COMPONENT_ROTARYENCODER_H_
#define INC_COMPONENT_ROTARYENCODER_H_

#include <main.h>

#include "private/Encoder.h"

class RotaryEncoder : public Encoder {
public:
	RotaryEncoder(TIM_HandleTypeDef* _tim_handle);
	virtual ~RotaryEncoder();

	void start();
	void stop();
	void updateAngle(); //一定間隔で呼び出したらいいかな
	int32_t getAngle() override;
	void setZero() override;

private:
	TIM_HandleTypeDef* tim_handle;
	int32_t total_count;
	uint32_t last_counter_value;
};

#endif /* INC_COMPONENT_ROTARYENCODER_H_ */
