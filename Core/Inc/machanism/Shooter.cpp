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
	if (locked) return false; //問題は発生していないが変更は認めない
	moving = move;
	if (moving) {
		move_ESC();
		move_Motor(ACCELOUT);
		shootertype = ShooterType::MOVE735;
		logger_.clear();
		spinFlag = false;
		lastRemain = 0;
		encoder->setZero();
		lastAngle = encoder->getAngle();
		ms_counter = 0;
		rotation_count = 0;
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
	ms_counter++;

	int32_t angle = encoder->getAngle();
	int16_t remain = angle % PULSE;

	if (shootertype != ShooterType::END) {
		if (ms_counter % 10 == 0) {
			int16_t speed = (angle - lastAngle) * 100;
			lastAngle = angle;

			logger_.push_back(speed);
			if (logger_.size() > LOGGERSIZE) {
				logger_.pop_front();
			}

			if (speed < SPEED735) {
				move_Motor(ACCELOUT);
			} else {
				move_Motor(MAINTATIONOUT);
			}
		}
	}

	if (remain < lastRemain) {
		spinFlag = false;
		rotation_count++;
	}

	if (rotation_count > MAXROTATION && remain > OPENPULSE && shootertype == ShooterType::MOVE735) {
		open_servo();
		stop_Motor();
		shootertype = ShooterType::STOPESC;
	}

	if (shootertype == ShooterType::MOVE735) {
		if (!spinFlag && remain > OPENPULSE) {
			auto valid_count = static_cast<std::size_t>(std::count_if(logger_.begin(), logger_.end(), [](int32_t s) {
				return s > (SPEED735 - 1000);
			}));

			if (logger_.size() >= static_cast<std::size_t>(LOGGERSIZE) && (valid_count * 10) >= (logger_.size() * 7)) {
				open_servo();
				stop_Motor();
				shootertype = ShooterType::STOPESC;
			}
			spinFlag = true;
		}
	} else if (shootertype == ShooterType::STOPESC) {
		if (remain >= ESCSTOPPULSE) {
			stop_ESC();
			shootertype = ShooterType::END;
		}
	}
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
