/*
 * UnitSterring.h
 * 操舵用のギアは1:4
 *  Created on: Jul 9, 2026
 *      Author: nika-
 */

#ifndef INC_MACHANISM_UNITSTEERING_H_
#define INC_MACHANISM_UNITSTEERING_H_

#include "main.h"
#include "m2006.h"
#include "MD4ch_child.h"
#include "PID.h"

enum setZeroMode{
	ROTATE180, //180°正転
	ROTATE360, //360°逆転
};

class Unit_Steering {
public:
	Unit_Steering(MD4ch_child* _drive, m2006* _steer, PID* _steer_pid);
	virtual ~Unit_Steering();

	void setZero();
	void InterruptZero();
	void Change_direction();
	void move(int16_t drive_value, int32_t steer_value);
	int32_t getFirstAngle(){ return firstAngle; }; //最初の位置を取得
	setZeroMode getZeromode(){ return zeromode; }; //現在のモードを取得、これをもとにエンコーダがどれくらい回転したらどういう動作をするかを

private:
	MD4ch_child* drive;
	m2006* steer;
	PID* steer_pid;

	bool settingZero;
	int32_t firstAngle; //原点取りをするために使う、180度回すときの最初の数値
	setZeroMode zeromode;
};

#endif /* INC_MACHANISM_UNITSTEERING_H_ */
