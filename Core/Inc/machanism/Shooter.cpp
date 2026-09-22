/*
 * Shooter.cpp
 *
 *  Created on: Jul 4, 2026
 *      Author: nika-
 */

#include "Shooter.h"

namespace {
    constexpr int16_t OPENCOUNT = 38;
    constexpr int16_t STOPCOUNT = 48;
    constexpr int16_t INCREASE_INTERVAL = 5;
    constexpr int16_t INCREASE_COUNT = OPENCOUNT / INCREASE_INTERVAL; // 7

    constexpr float MOTOR735_FIRST = -80.0f;
    constexpr float MOTOR735_MAX = -145.0f;
    constexpr float MOTOR735_INCREASE = (MOTOR735_MAX - MOTOR735_FIRST) / INCREASE_COUNT;

    constexpr float MOTOR385_FIRST = -80.0f;
    constexpr float MOTOR385_MAX = -150.0f;
    constexpr float MOTOR385_INCREASE = (MOTOR385_MAX - MOTOR385_FIRST) / INCREASE_COUNT;

    constexpr uint16_t SERVO_0 = 1000;
    constexpr uint16_t SERVO_180 = 2000;
    constexpr uint16_t SERVO_ANGLE_CLOSE = 165;
    constexpr uint16_t SERVO_ANGLE_OPEN = 90;
}

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
	if (locked) return true; //問題は発生していない
	moving = move;
	if (moving) {
		move_Motor735(static_cast<int16_t>(MOTOR735_FIRST));
		move_Motor385(static_cast<int16_t>(MOTOR385_FIRST));
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
			move_Motor385(static_cast<int16_t>(MOTOR385_FIRST + quotient * MOTOR385_INCREASE));
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
