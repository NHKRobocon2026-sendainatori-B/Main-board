/*
 * UnitSterring.cpp
 *
 *  Created on: Jul 9, 2026
 *      Author: nika-
 */

#include <UnitSteering.h>

/*独ステの1ユニット*/
Unit_Steering::Unit_Steering(MD4ch_child* _drive, m2006* _steer, PID* _steer_pid)
: drive(_drive), steer(_steer), steer_pid(_steer_pid), settingZero(false), firstAngle(_steer->getAngle())
{
	// TODO Auto-generated constructor stub
	drive->setMode(Mode::OPENLOOP);
}

Unit_Steering::~Unit_Steering() {
	// TODO Auto-generated destructor stub
}

/*原点を取る*/
void Unit_Steering::setZero(){
	if (!settingZero) return;
	settingZero = true;
	firstAngle = steer->getAngle();
	zeromode = setZeroMode::ROTATE180;

	steer_pid->disable();
	steer->move(3000);
}

/*180°回ったとしても見つからなかった。向き変更*/
void Unit_Steering::Change_direction(){
	zeromode = setZeroMode::ROTATE360;
	steer->move(-3000);
}

/*フォトインタラプタからの割り込みで呼び出す、PIDを有効にして*/
void Unit_Steering::InterruptZero(){
	if (!settingZero) return;
	steer->move(0);
	steer->setZero();
	steer_pid->enable();
	settingZero = false;
}

/*出力する*/
void Unit_Steering::move(int16_t drive_value, int32_t steer_value){
	if (settingZero) return;
	drive->setOut(drive_value);
	steer_pid->setTarget(steer_value);
}
