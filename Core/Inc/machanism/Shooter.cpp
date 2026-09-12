/*
 * Shooter.cpp
 *
 *  Created on: Jul 4, 2026
 *      Author: nika-
 */

#include "Shooter.h"

#define MOTOR_OUT -120 //モーターのノーマル出力、絶対変更
#define AGAIN_OUT -25 //再スタートを待つ速度
#define ESC_OUT 100 //ESCの出力、絶対変更
#define ESC_MAX 90
#define SERVO_0 1000
#define SERVO_180 2000
#define SERVO_ANGLE_CLOSE 158 //サーボのアングル、絶対変更
#define SERVO_ANGLE_OPEN 100 //サーボのアングル開いたとき、絶対変更

Shooter::Shooter(MD4ch_child* _motor, ESC* _esc, Servo* _servo, Steering* _steer)
: motor(_motor), esc(_esc), servo(_servo), _steer(_steer)
{
	// TODO Auto-generated constructor stub
}

Shooter::~Shooter() {
	// TODO Auto-generated destructor stub
}

void Shooter::init(){
	motor->setMode(Mode::OPENLOOP);
	esc->setMax(ESC_MAX);
	servo->setting(SERVO_0, SERVO_180);
}

bool Shooter::moveShooter() {
	if (locked || moving) return false;
	moving = true;
	if (moving) {
		move_ESC();
		move_Motor(MOTOR_OUT);
	}
	return true;
}
bool Shooter::grab() {
	if (locked || moving) return false;
	close_servo();
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

//フォトインタラプタの割り込み
void Shooter::Interrupt() {
	if (!moving) return; //手動で回っている等射出には関係なし
	counter++;
	if (counter == 14) {
		open_servo();
		move_Motor(AGAIN_OUT);
	} else if (counter == 19) {
		stop_ESC();
	} else if (counter == 30) {
		stop_Motor();
		_steer->shooterMode(false);
		counter = 0;
		moving = false;
	}
}

void Shooter::lock(){
	motor->lock();
	esc->lock();
	servo->lock();
	counter = 0;
	locked = true;
}

void Shooter::unlock(){
	motor->unlock();
	esc->unlock();
	servo->unlock();
	locked = false;
}
