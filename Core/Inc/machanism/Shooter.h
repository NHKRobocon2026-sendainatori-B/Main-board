/*
 * Shooter.h
 *
 *  Created on: Jul 4, 2026
 *      Author: nika-
 */

#ifndef INC_MACHANISM_SHOOTER_H_
#define INC_MACHANISM_SHOOTER_H_

#include "main.h"
#include "MD4ch_child.h"
#include "Servo.h"

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

	void lock();
	void unlock();

private:
	MD4ch_child* motor735; //真ん中で振り回す場所
	MD4ch_child* motor385; //先端の385
	Servo* servo; //雑巾を掴むサーボ

	uint16_t counter = 0; //フォトインタラプタの割り込みカウンタ

	bool moving = false;
	bool locked = false;
};

#endif /* INC_MACHANISM_SHOOTER_H_ */
