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
#include "PositionPIDController.h"

enum setZeroMode{
	ROTATE180, //180°正転
	ROTATE360, //360°逆転
	SETERROR
};

class UnitSteering {
public:
	UnitSteering(MD4ch_child* _drive, PositionPIDController* _steer_pid);
	virtual ~UnitSteering();

	void init();

	void setZero();
	void InterruptZero();
	void Change_direction();
	void move(int16_t drive_value, int32_t steer_value);
	int32_t getFirstAngle(){ return firstAngle; }; //最初の位置を取得
	int32_t getAngle(){ return steer_pid->speed_pid_->enc->getAngle(); }
	bool isSettingZero() { return  settingZero; };
	setZeroMode getZeromode(){ return zeromode; }; //現在のモードを取得、これをもとにエンコーダがどれくらい回転したらどういう動作をするかを
	PositionPIDController* getSteerPID() { return steer_pid; };

	void lock();
	void unlock();

private:
	MD4ch_child* drive;
	PositionPIDController* steer_pid;

	bool settingZero = false;
	int32_t firstAngle = 0; //原点取りをするために使う、180度回すときの最初の数値
	setZeroMode zeromode = setZeroMode::ROTATE180;

	bool locked = true;
};

#endif /* INC_MACHANISM_UNITSTEERING_H_ */
