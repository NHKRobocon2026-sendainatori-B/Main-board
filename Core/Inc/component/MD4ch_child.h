/*
 * Ch4child.h
 *
 * 開発してもらった4chMDの子クラスです。モータのモード・出力はこちらを変更します
 *
 *  Created on: Jun 19, 2026
 *      Author: nika-
 */

#ifndef INC_COMPONENT_MD4CH_CHILD_H_
#define INC_COMPONENT_MD4CH_CHILD_H_

#include "main.h"
#include "private/Actuator.h"

enum class Mode{
	STOP,
	POSITION,
	OPENLOOP,
	ENCODER
};

class MD4ch_child : Actuator{
public:
	MD4ch_child();
	virtual ~MD4ch_child();

	void setMode(Mode _mode);
	void setOut(int16_t _out);
	void move(int16_t _out) override;
	Mode getMode();
	int16_t getOut();

	void lock() override;
	void unlock() override;

private:
	Mode mode = Mode::OPENLOOP;
	Mode lastmode; //lock解除後にmodeを戻すための変数
	int16_t out = 0;
	bool locked = false;
};



#endif /* INC_COMPONENT_MD4CH_CHILD_H_ */
