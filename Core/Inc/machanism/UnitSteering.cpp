/*
 * UnitSterring.cpp
 *
 *  Created on: Jul 9, 2026
 *      Author: nika-
 */

#include <UnitSteering.h>

/*独ステの1ユニット*/
UnitSteering::UnitSteering(MD4ch_child* _drive, PositionPIDController* _steer_pid)
: drive(_drive), steer_pid(_steer_pid), settingZero(false), firstAngle(_steer_pid->speed_pid_->enc->getAngle())
{
	// TODO Auto-generated constructor stub
	drive->setMode(Mode::OPENLOOP);
}

UnitSteering::~UnitSteering() {
	// TODO Auto-generated destructor stub
}

/*原点を取る*/
void UnitSteering::setZero(){
	settingZero = true;
	firstAngle = steer_pid->speed_pid_->enc->getAngle();
	zeromode = setZeroMode::ROTATE180;

	steer_pid->disable();
	steer_pid->speed_pid_->setTarget(3000);
}

/*180°回ったとしても見つからなかった。向き変更*/
void UnitSteering::Change_direction(){
	if (!settingZero) return;
	if (zeromode == setZeroMode::ROTATE180){
		zeromode = setZeroMode::ROTATE360;
		steer_pid->speed_pid_->setTarget(-3000);
	} else if (zeromode == setZeroMode::ROTATE360){
		zeromode = setZeroMode::SETERROR;
		steer_pid->lock();
		drive->lock();
	}
}

/*フォトインタラプタからの割り込みで呼び出す、PIDを有効にして*/
void UnitSteering::InterruptZero(){
	if (!settingZero) return;
	steer_pid->speed_pid_->setTarget(0);
	steer_pid->speed_pid_->enc->setZero();
	steer_pid->enable();
	steer_pid->setTarget(0);
	settingZero = false;
}

/*出力する*/
void UnitSteering::move(int16_t drive_value, int32_t steer_value){
	if (settingZero) return;
	if (locked) return;
	drive->setOut(drive_value);
	steer_pid->setTarget(steer_value);
}

void UnitSteering::lock(){
	steer_pid->lock();
	drive->lock();
	locked = true;
}

void UnitSteering::unlock(){
	steer_pid->unlock();
	drive->unlock();
	locked = false;
}
