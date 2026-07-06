/*
 * Actuator.h
 *
 *  Created on: Jul 2, 2026
 *      Author: nika-
 */

#ifndef INC_COMPONENT_PRIVATE_ACTUATOR_H_
#define INC_COMPONENT_PRIVATE_ACTUATOR_H_

#include "main.h"

class Actuator {
public:
	Actuator();
	virtual ~Actuator();

	/* 動かす */
	virtual void move(int16_t out) = 0;
	/* ロック */
	virtual void lock() = 0;
	/* アンロック */
	virtual void unlock() = 0;
};

#endif /* INC_COMPONENT_PRIVATE_ACTUATOR_H_ */
