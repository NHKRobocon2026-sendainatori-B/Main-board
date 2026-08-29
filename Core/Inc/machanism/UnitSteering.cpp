/*
 * UnitSterring.cpp
 *
 *  Created on: Jul 9, 2026
 *      Author: nika-
 */

#include <UnitSteering.h>

#define STEER_MAXCURRENT 6000
#define SPEED_ALLOWERROR 0
#define SPEED_INTEGRAL 1000
#define SPEED_INTERVAL 5
#define SPEED_P 1.0f
#define SPEED_I 0.02f
#define SPEED_D 0.01f

#define POSITION_ALLOWERROR 1000
#define POSITION_ACCEL 50000
#define POSITION_INTEGRAL 1000
#define POSITION_INTERVAL 10
#define POSITION_MAXSPEED 9000
#define POSITION_P 0.09f
#define POSITION_I 0.14f
#define POSITION_D 0.015f

#define SETZERO_SPEED (POSITION_MAXSPEED / 2)

/*独ステの1ユニット*/
UnitSteering::UnitSteering(MD4ch_child* _drive, PositionPIDController* _steer_pid)
: drive(_drive), steer_pid(_steer_pid), settingZero(false), firstAngle(_steer_pid->speed_pid_->enc->getAngle())
{
	// TODO Auto-generated constructor stub

}

UnitSteering::~UnitSteering() {
	// TODO Auto-generated destructor stub
}

void UnitSteering::init(){
	drive->setMode(Mode::OPENLOOP);
	steer_pid->speed_pid_->setMaxOutput(STEER_MAXCURRENT);
	steer_pid->speed_pid_->setAllowError(SPEED_ALLOWERROR);
	steer_pid->speed_pid_->setMaxIntegral(SPEED_INTEGRAL);
	steer_pid->speed_pid_->setInterval(SPEED_INTERVAL);
	steer_pid->speed_pid_->setPID(SPEED_P, SPEED_I, SPEED_D);
	steer_pid->setAllowError(POSITION_ALLOWERROR);
	steer_pid->setMaxAcceleration(POSITION_ACCEL);
	steer_pid->setMaxIntegral(POSITION_INTEGRAL);
	steer_pid->setInterval(POSITION_INTERVAL);
	steer_pid->setMaxSpeed(POSITION_MAXSPEED);
	steer_pid->setPID(POSITION_P, POSITION_I, POSITION_D);
}

/*原点を取る*/
void UnitSteering::setZero(){
	settingZero = true;
	firstAngle = steer_pid->speed_pid_->enc->getAngle();
	zeromode = setZeroMode::ROTATE180;

	steer_pid->disable();
	steer_pid->speed_pid_->setTarget(SETZERO_SPEED);
}

/*180°回ったとしても見つからなかった。向き変更*/
void UnitSteering::Change_direction(){
	if (!settingZero) return;
	if (zeromode == setZeroMode::ROTATE180){
		zeromode = setZeroMode::ROTATE360;
		steer_pid->speed_pid_->setTarget(-SETZERO_SPEED);
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
