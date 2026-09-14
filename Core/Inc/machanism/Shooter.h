/*
 * Shooter.h
 *
 *  Created on: Jul 4, 2026
 *      Author: nika-
 */

#ifndef INC_MACHANISM_SHOOTER_H_
#define INC_MACHANISM_SHOOTER_H_

#include <deque>
#include <algorithm>

#include "main.h"
#include "MD4ch_child.h"
#include "ESC.h"
#include "Servo.h"
#include "RotaryEncoder.h"

#define SHOOTER_SERVO_0 1000
#define SHOOTER_SERVO_180 2000
#define PULSE 2048 //一周のパルス
#define OPENPULSE (PULSE * 0.15) //射出地点のパルス
#define ESCSTOPPULSE (PULSE * 0.5)
#define SPEED735 (PULSE * 1.5) //１秒で何回転したいか
#define ACCELOUT -110
#define MAINTATIONOUT -25
#define LOGGERSIZE 50

#define MOTOR_OUT -120 //モーターのノーマル出力、絶対変更
#define AGAIN_OUT -25 //再スタートを待つ速度
#define ESC_OUT 100 //ESCの出力、絶対変更
#define ESC_MAX 90
#define SERVO_0 1000
#define SERVO_180 2000
#define SERVO_ANGLE_CLOSE 158 //サーボのアングル、絶対変更
#define SERVO_ANGLE_OPEN 100 //サーボのアングル開いたとき、絶対変更

class Shooter {
public:
	Shooter(MD4ch_child* _motor, ESC* _esc, Servo* _servo, RotaryEncoder* _encoder);
	virtual ~Shooter();

	void init();

	bool moveShooter(bool move);

	void move_Motor(int16_t out);
	void stop_Motor();
	void move_ESC();
	void stop_ESC();
	void open_servo();
	void close_servo();
	void Interrupt();

	void lock();
	void unlock();

	MD4ch_child* get_motor() { return motor; };
	ESC* get_esc() { return esc; };

private:
	MD4ch_child* motor; //真ん中で振り回す場所
	ESC* esc; //端で雑巾をぐるぐる回す場所
	Servo* servo; //雑巾を掴むサーボ
	RotaryEncoder* encoder; //計測するエンコーダー

	int32_t lastAngle = 0;
	int16_t lastRemain = 0;
	enum ShooterType {
		MOVE735,
		STOPESC
	} shootertype = ShooterType::MOVE735;
	std::deque<int32_t> logger_;

	bool moving = false;
	bool locked = false;
	bool spinFlag = false;
};

#endif /* INC_MACHANISM_SHOOTER_H_ */
