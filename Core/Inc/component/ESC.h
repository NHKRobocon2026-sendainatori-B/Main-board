/*
 * ESC.h
 *
 *  Created on: Jul 11, 2026
 *      Author: nika-
 */

#ifndef INC_COMPONENT_ESC_H_
#define INC_COMPONENT_ESC_H_

#include "main.h"

class ESC {
public:
	ESC(TIM_HandleTypeDef* _tim_handle, uint16_t _tim_channel);
	virtual ~ESC();

	void setMax(uint16_t _maxOut);

	void move(uint8_t ratio);

	void lock();
	void unlock();
private:
	TIM_HandleTypeDef* tim_handle;
	uint16_t tim_channel;
	uint16_t maxOut;

	bool locked;
};

#endif /* INC_COMPONENT_ESC_H_ */
