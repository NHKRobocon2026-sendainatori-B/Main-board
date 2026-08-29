/*
 * Odometry.cpp
 *
 *  Created on: Jul 4, 2026
 *      Author: nika-
 */

#include "Odometry.h"

Odometry::Odometry(RotaryEncoder* encoderX_, RotaryEncoder* encoderY_)
: encoderX_(encoderX_), encoderY_(encoderY_)
{
	// TODO Auto-generated constructor stub

}

Odometry::~Odometry() {
	// TODO Auto-generated destructor stub
}

void Odometry::setZero() {
	encoderX_->setZero();
	encoderY_->setZero();
	lastAngle[0] = 0;
	lastAngle[1] = 0;
}

std::array<int32_t, 2> Odometry::getTransformation(){
	auto array = std::array<int32_t, 2>();

	int32_t nowX = encoderX_->getAngle();
	int32_t nowY = encoderY_->getAngle();

	array[0] = nowX - lastAngle[0];
	array[1] = nowY - lastAngle[1];

	lastAngle[0] = nowX;
	lastAngle[1] = nowY;

	return array;
}
