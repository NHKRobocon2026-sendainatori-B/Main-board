/*
 * Shooter.cpp
 *
 *  Created on: Jul 4, 2026
 *      Author: nika-
 */

#include "Shooter.h"

Shooter::Shooter(MD4ch_child* _motor, ESC* _esc, Servo* _servo)
: motor(_motor), esc(_esc), servo(_servo),counter(0), locked(false)
{
	// TODO Auto-generated constructor stub
	motor->setMode(Mode::OPENLOOP);
}

Shooter::~Shooter() {
	// TODO Auto-generated destructor stub
}

void Shooter::move_Motor(){
	if (locked) return;
	motor->setOut(motor_out);
}

void Shooter::stop_Motor(){
	motor->setOut(0);
}

void Shooter::move_ESC(){
	if (locked) return;
	esc->move(10);
}

void Shooter::stop_ESC(){
	esc->move(0);
}

void Shooter::open_servo(){
	if (locked) return;
	servo->move(servo_angle);
}

void Shooter::close_servo(){
	if (locked) return;
	servo->move(0);
}

void Shooter::Interrupt(){
	counter++;
	if (counter % 3 == 0){
		//何らかの動作
	}
}

void Shooter::lock(){
	motor->lock();
	esc->lock();
	locked = true;
}

void Shooter::unlock(){
	motor->unlock();
	esc->unlock();
	locked = false;
}
