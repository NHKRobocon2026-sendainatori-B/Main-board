/*
 * Shooter.cpp
 *
 *  Created on: Jul 4, 2026
 *      Author: nika-
 */

#include "Shooter.h"

Shooter::Shooter(MD4ch_child* _motor, ESC* _esc, Servo* _servo, RotaryEncoder* _encoder)
: motor(_motor), esc(_esc), servo(_servo), encoder(_encoder)
{
	// TODO Auto-generated constructor stub
	encoder->start();
}

Shooter::~Shooter() {
	// TODO Auto-generated destructor stub
}

void Shooter::init(){
	motor->setMode(Mode::OPENLOOP);
	esc->setMax(ESC_MAX);
	servo->setting(SERVO_0, SERVO_180);
}

bool Shooter::moveShooter(bool move) {
	if (locked) return true; //問題は発生していない
	moving = move;
	if (moving) {
		move_ESC();
		move_Motor(MOTOR_OUT);
		shootertype = ShooterType::MOVE735;
		logger_.clear();
		spinFlag = false;
		lastRemain = 0;
		lastAngle = 0;
		encoder->setZero();
	}
	return true;
}

void Shooter::move_Motor(int16_t out){
	if (locked) return;
	motor->setOut(out);
}

void Shooter::stop_Motor(){
	motor->setOut(0);
}

void Shooter::move_ESC(){
	if (locked) return;
	esc->move(ESC_MAX);
}

void Shooter::stop_ESC(){
	esc->move(0);
}

void Shooter::open_servo(){
	if (locked) return;
	servo->move(SERVO_ANGLE_OPEN);
}

void Shooter::close_servo(){
	if (locked) return;
	servo->move(SERVO_ANGLE_CLOSE);
}

//1msごとの割り込み
void Shooter::Interrupt() {
	if (!moving) return; //手動で回っている等射出には関係なし

	int32_t angle = encoder->getAngle();
	int16_t remain = angle % PULSE;
	if (remain < lastRemain) {
		spinFlag = false;
	}

	switch (shootertype){
	case ShooterType::MOVE735:
		int32_t speed = (angle - lastAngle) * 1000;
		logger_.push_back(speed);
		if (logger_.size() > LOGGERSIZE) logger_.pop_back();
		move_Motor((speed < SPEED735) ? ACCELOUT : MAINTATIONOUT);
		if (!spinFlag && remain > OPENPULSE) {
			auto valid_count = static_cast<std::size_t>(std::count_if(logger_.begin(), logger_.end(), [](int32_t speed){
				return speed > (SPEED735 - 1000);
			}));
			if (logger_.size() >= static_cast<std::size_t>(LOGGERSIZE) && (valid_count * 10) >= (logger_.size() * 5)) {
				open_servo();
				stop_Motor();
				shootertype = ShooterType::STOPESC;
			}
			spinFlag = true;
		}
		break;

	case ShooterType::STOPESC:
		if (remain < ESCSTOPPULSE) break;
		stop_ESC();
		spinFlag = false;
		moving = false;
		break;
	}
	lastRemain = remain;
}

void Shooter::lock(){
	motor->lock();
	esc->lock();
	servo->lock();
	moving = false;
	locked = true;
}

void Shooter::unlock(){
	motor->unlock();
	esc->unlock();
	servo->unlock();
	locked = false;
}
