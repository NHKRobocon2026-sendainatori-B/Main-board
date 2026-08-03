/*
 * Steering.h
 *
 *  Created on: Jul 4, 2026
 *      Author: nika-
 */

#ifndef INC_MACHANISM_STEERING_H_
#define INC_MACHANISM_STEERING_H_

#include <array>
#include "main.h"
#include "UnitSteering.h"

class Steering {
public:
	Steering(std::array<UnitSteering*, 4>* _units);
	virtual ~Steering();

	void move(int16_t x, int16_t y, int16_t theta);
	void setZero();

	void lock();
	void unlock();
private:
	std::array<UnitSteering*, 4>* units;

	bool settingZero = false;
	bool locked = false;
};

#endif /* INC_MACHANISM_STEERING_H_ */
