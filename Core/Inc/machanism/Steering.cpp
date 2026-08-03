/*
 * Steering.cpp
 *
 *  Created on: Jul 4, 2026
 *      Author: nika-
 */

#include "Steering.h"

Steering::Steering(std::array<UnitSteering*, 4>* _units)
: units(_units)
{
	// TODO Auto-generated constructor stub

}

Steering::~Steering() {
	// TODO Auto-generated destructor stub
}

void Steering::move(int16_t x, int16_t y, int16_t theta){
	if (locked) return;
	//計算をここに
}

void Steering::setZero(){
	for (auto unit : *units){
		unit->setZero();
	}
}

void Steering::lock(){
	locked = true;
	for (auto unit : *units){
		unit->lock();
	}
}

void Steering::unlock(){
	locked = false;
	for (auto unit : *units){
		unit->unlock();
	}
}
