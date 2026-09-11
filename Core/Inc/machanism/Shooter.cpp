/*
 * Shooter.cpp
 *
 *  Created on: Jul 4, 2026
 *      Author: nika-
 */

#include "Shooter.h"

#define MOTOR_OUT -40 //モーターのノーマル出力、絶対変更
#define LOAD_OUT -25 //装填時の速度
#define AGAIN_OUT -25 //再スタートを待つ速度
#define ESC_OUT 100 //ESCの出力、絶対変更
#define ESC_MAX 90
#define SERVO_0 1000
#define SERVO_180 2000
#define SERVO_ANGLE_CLOSE 150 //サーボのアングル、絶対変更
#define SERVO_ANGLE_OPEN 100 //サーボのアングル開いたとき、絶対変更

Shooter::Shooter(MD4ch_child* _motor, ESC* _esc, Servo* _servo)
: motor(_motor), esc(_esc), servo(_servo)
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
	stop_ESC();
}

bool Shooter::moveShooter(bool move){
	//装填のモード変更
	//もし失敗したら下のtrueを変化
	moving = move;
	if (!move) {
		again_flag = true;
		move_Motor(AGAIN_OUT);
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

void Shooter::Interrupt(){
	if (again_flag){
		if (counter % 6 == 0) {
			move_Motor(LOAD_OUT);
			again_flag = false;
		}
	} else {
		if (counter % 6 == 0) {
			move_Motor(LOAD_OUT);
		}
		if (counter % 6 == 1) {
			if (start_flag) {
				open_servo();
				start_flag = false;
			}
			//装填を上にあげる
		}
		if (counter % 6 == 2) {
			close_servo();
		}
		if (counter % 6 == 3) {
			move_ESC();
			move_Motor(MOTOR_OUT);
		}
		if (counter % 6 == 5) {
			open_servo();
			stop_ESC();
		}
	}
	counter++;
}

void Shooter::lock(){
	motor->lock();
	esc->lock();
	servo->lock();
	locked = true;
}

void Shooter::unlock(){
	motor->unlock();
	esc->unlock();
	servo->unlock();
	locked = false;
}
