/*
 * Shooter.cpp
 *
 *  Created on: Jul 4, 2026
 *      Author: nika-
 */

#include "Shooter.h"

Shooter::Shooter(MD4ch_child* _motor735, MD4ch_child* _motor385, Servo* _servo)
: motor735(_motor735), motor385(_motor385), servo(_servo)
{
	// TODO Auto-generated constructor stub
}

Shooter::~Shooter() {
	// TODO Auto-generated destructor stub
}

void Shooter::init(){
	motor735->setMode(Mode::OPENLOOP);
	motor385->setMode(Mode::OPENLOOP);
	servo->setting(SERVO_0, SERVO_180);
}

bool Shooter::moveShooter(bool move) {
	if (locked) return false; //問題は発生していない
	moving = move;
	if (moving) {
		move_Motor735(static_cast<int16_t>(MOTOR735_FIRST));
		if (mode != ShooterMode::BUCKET) move_Motor385(static_cast<int16_t>(MOTOR385_FIRST));
	}
	return true;
}

void Shooter::move_Motor735(int16_t out){
	if (locked) return;
	motor735->setOut(out);
}

void Shooter::stop_Motor735(){
	motor735->setOut(0);
}

void Shooter::move_Motor385(int16_t out) {
	if (locked) return;
	motor385->setOut(out);
}

void Shooter::stop_Motor385(){
	motor385->setOut(0);
}

void Shooter::open_servo(){
	if (locked) return;
	servo->move(SERVO_ANGLE_OPEN);
}

void Shooter::close_servo(){
	if (locked) return;
	servo->move(SERVO_ANGLE_CLOSE);
}

//フォトインタラプタの割り込み
void Shooter::Interrupt() {
	if (!moving) return; //手動で回っている等射出には関係なし
	counter++;
	if (counter % INCREASE_INTERVAL == 0) {
		int16_t quotient = counter / INCREASE_INTERVAL;
		if (quotient <= INCREASE_COUNT) {
			move_Motor735(static_cast<int16_t>(MOTOR735_FIRST + quotient * MOTOR735_INCREASE));
			if (mode != ShooterMode::BUCKET) move_Motor385(static_cast<int16_t>(MOTOR385_FIRST + quotient * MOTOR385_INCREASE));
		}
	}
	if (counter == OPENCOUNT) {
		open_servo();
	}
	if (counter == STOPCOUNT) {
		stop_Motor735();
		stop_Motor385();
		moving = false;
		counter = 0;
	}
}

void Shooter::variableUpdate(std::array<uint8_t, 5> data) {
	if (moving) return;

	MOTOR385_FIRST = static_cast<float>(data[0] * -1.0f);
	MOTOR385_MAX = static_cast<float>(data[1] * -1.0f);
	MOTOR735_FIRST = static_cast<float>(data[2] * -1.0f);
	MOTOR735_MAX = static_cast<float>(data[3] * -1.0f);
	OPENCOUNT = data[4];

	INCREASE_COUNT = OPENCOUNT / INCREASE_INTERVAL;
	STOPCOUNT = OPENCOUNT + 10;

	MOTOR385_INCREASE = (MOTOR385_MAX - MOTOR385_FIRST) / INCREASE_COUNT;
	MOTOR735_INCREASE = (MOTOR735_MAX - MOTOR735_FIRST) / INCREASE_COUNT;
}

bool Shooter::setMode(uint8_t data) {
	if (moving) return false;

	if (data == 0) {
		mode = ShooterMode::FLAG;
		OPENCOUNT = 41;
	} else if (data == 1) {
		mode = ShooterMode::DESK;
		OPENCOUNT = 39;
	} else if (data == 2) {
		mode = ShooterMode::BUCKET;
		OPENCOUNT = 41;
	} else {
		return false;
	}
	INCREASE_COUNT = OPENCOUNT / INCREASE_INTERVAL;
	STOPCOUNT = OPENCOUNT + 10;

	MOTOR385_INCREASE = (MOTOR385_MAX - MOTOR385_FIRST) / INCREASE_COUNT;
	MOTOR735_INCREASE = (MOTOR735_MAX - MOTOR735_FIRST) / INCREASE_COUNT;

	return true;
}

void Shooter::lock(){
	motor735->lock();
	motor385->lock();
	servo->lock();
	counter = 0;
	locked = true;
}

void Shooter::unlock(){
	motor735->unlock();
	motor385->unlock();
	servo->unlock();
	locked = false;
}
