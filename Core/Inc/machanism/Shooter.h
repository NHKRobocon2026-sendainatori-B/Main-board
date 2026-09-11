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
#include "ESC.h"
#include "Servo.h"

class Shooter {
public:
	Shooter(MD4ch_child* _motor, ESC* _esc, Servo* _servo);
	virtual ~Shooter();

	void init();

	bool moveShooter();
	bool grab();

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

	uint16_t counter = 0; //フォトインタラプタの割り込みカウンタ

	bool moving = false;
	bool locked = false;
};

#endif /* INC_MACHANISM_SHOOTER_H_ */
