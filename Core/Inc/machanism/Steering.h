/*
 * Steering.h
 *
 *  Created on: Jul 4, 2026
 *      Author: nika-
 */

#ifndef INC_MACHANISM_STEERING_H_
#define INC_MACHANISM_STEERING_H_

#include <array>
#include <cmath>
#include <map>
#include <utility>
#include <algorithm>
#include "main.h"
#include "UnitSteering.h"
#include "EmbeddedPromise.hpp"

struct Target {
	UnitSteering* steer;
	Promise<bool> promise;
};

enum class ProcessStatus {
    IN_PROGRESS,
    SUCCESS,
    FAILED
};

class Steering {
public:
	Steering(std::array<UnitSteering*, 4>* _units, std::array<uint16_t, 4>* interrupts);
	virtual ~Steering();

	void init();

	void move(float x, float y, float yaw);
	void setZero();
	ProcessStatus interruptsetZero(uint16_t key);
	ProcessStatus setZerocheckStatus();
	ProcessStatus setZeroupdate();

	void updateSpeed();
	void updatePosition();

	void lock();
	void unlock();
private:
	std::array<UnitSteering*, 4>* units;

	std::array<uint16_t, 4>* interrupts;
	std::array<Future<bool>, 4> futures;
	std::map<uint16_t, Target> pending_promises_;

	bool settingZero = false;
	bool locked = false;
};

#endif /* INC_MACHANISM_STEERING_H_ */
