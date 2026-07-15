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

class Shooter {
public:
	Shooter(MD4ch_child* _motor, ESC* _esc);
	virtual ~Shooter();

	void move_Motor();
	void stop_Motor();
	void move_ESC();
	void stop_ESC();
	void Interrupt();

	void lock();
	void unlock();

	MD4ch_child* get_motor() { return motor; };
	ESC* get_esc() { return esc; };

private:
	MD4ch_child* motor; //真ん中で振り回す場所
	ESC* esc; //端で雑巾をぐるぐる回す場所

	uint16_t counter; //フォトインタラプタの割り込みカウンタ

	const int16_t motor_out = 300; //モーターの出力、絶対変更
	const int8_t esc_out = 5; //ESCの出力、絶対変更

	bool locked;
};

#endif /* INC_MACHANISM_SHOOTER_H_ */
