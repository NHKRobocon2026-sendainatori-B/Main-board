/*
 * Odometry.h
 *
 *  Created on: Jul 4, 2026
 *      Author: nika-
 */

#ifndef INC_MACHANISM_ODOMETRY_H_
#define INC_MACHANISM_ODOMETRY_H_

#include <array>
#include "main.h"
#include "RotaryEncoder.h"

class Odometry {
public:
	Odometry(RotaryEncoder* encoderX_, RotaryEncoder* encoderY_);
	virtual ~Odometry();
	std::array<int32_t, 2> getTransformation();
	void setZero();

private:
	RotaryEncoder* encoderX_;
	RotaryEncoder* encoderY_;
	std::array<int32_t, 2> lastAngle = {0, 0};
};

#endif /* INC_MACHANISM_ODOMETRY_H_ */
