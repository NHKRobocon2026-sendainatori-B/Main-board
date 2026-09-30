/*
 * Shooter.h
 *
 *  Created on: Jul 4, 2026
 *      Author: nika-
 */

#ifndef INC_MACHANISM_SHOOTER_H_
#define INC_MACHANISM_SHOOTER_H_

#include "array"

#include "main.h"
#include "MD4ch_child.h"
#include "Servo.h"

enum ShooterMode {
	FLAG,
	DESK,
	BUCKET
};

class Shooter {
public:
	Shooter(MD4ch_child* _motor735, MD4ch_child* _motor385, Servo* _servo);
	virtual ~Shooter();

	void init();

	bool moveShooter(bool move);

	void move_Motor735(int16_t out);
	void stop_Motor735();
	void move_Motor385(int16_t out);
	void stop_Motor385();
	void open_servo();
	void close_servo();
	void Interrupt();

	void variableUpdate(std::array<uint8_t, 5> data);
	bool setMode(uint8_t data);

	void lock();
	void unlock();

private:
	MD4ch_child* motor735; //真ん中で振り回す場所
	MD4ch_child* motor385; //先端の385
	Servo* servo; //雑巾を掴むサーボ
	ShooterMode mode = ShooterMode::FLAG;

	uint16_t counter = 0; //フォトインタラプタの割り込みカウンタ

	bool moving = false;
	bool locked = false;

	int16_t OPENCOUNT = 41;
	int16_t STOPCOUNT = 51;
	int16_t INCREASE_INTERVAL = 5;
	int16_t INCREASE_COUNT = OPENCOUNT / INCREASE_INTERVAL; // 7

	float MOTOR735_FIRST = -80.0f;
	float MOTOR735_MAX = -145.0f;
	float MOTOR735_INCREASE = (MOTOR735_MAX - MOTOR735_FIRST) / INCREASE_COUNT;

	float MOTOR385_FIRST = -80.0f;
	float MOTOR385_MAX = -150.0f;
	float MOTOR385_INCREASE = (MOTOR385_MAX - MOTOR385_FIRST) / INCREASE_COUNT;

	uint16_t SERVO_0 = 1000;
	uint16_t SERVO_180 = 2000;
	uint16_t SERVO_ANGLE_CLOSE = 43;
	uint16_t SERVO_ANGLE_OPEN = 2;
};

#endif /* INC_MACHANISM_SHOOTER_H_ */
