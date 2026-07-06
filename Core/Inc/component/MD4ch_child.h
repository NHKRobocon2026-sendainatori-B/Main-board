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

enum class Mode{
	STOP,
	POSITION,
	OPENLOOP,
	ENCODER
};

class MD4ch_child{
public:
	MD4ch_child();
	virtual ~MD4ch_child();

	void setMode(Mode _mode);
	void setOut(int16_t _out);
	Mode getMode();
	int16_t getOut();

	void lock();
	void unlock();

private:
	Mode mode;
	int16_t out;
	bool locked;
};



#endif /* INC_COMPONENT_MD4CH_CHILD_H_ */
