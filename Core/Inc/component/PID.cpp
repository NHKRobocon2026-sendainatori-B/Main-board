/*
 * PID.cpp
 *
 *  Created on: Jul 4, 2026
 *      Author: nika-
 */

#include "component/PID.h"

//PID(位置制御)を扱える
PID::PID(Actuator* _act, Encoder* _enc)
: act(_act), enc(_enc), target(0), kp(0.1f), ki(0.1f), kd(0.1f),
  max_integral(0.0f), integral(0.0f), last_error(0.0f), dt(10), allowError(10), max_output(4096),
  locked(false), effective(true)
{
	// TODO Auto-generated constructor stub

}

PID::~PID() {
	// TODO Auto-generated destructor stub
}

/* 目標値を設定、この位置になるようにする */
void PID::setTarget(int32_t _target){
	target = _target;
	reset();
}

/* pidの値を設定 */
void PID::setPID(float _kp, float _ki, float _kd){
	kp = _kp;
	ki = _ki;
	kd = _kd;
}

/* 許容誤差を設定 */
void PID::setAllowError(int16_t _allowError){
	allowError = _allowError;
}

/* 積分の最大値を設定 */
void PID::setMaxIntegral(float _max_integral){
	max_integral = _max_integral;
}

/* 最大出力を設定 */
void PID::setMaxOutput(int16_t _max_output){
	max_output = _max_output;
}

/* calculatePIDが呼び出される間隔を設定、単位はms */
void PID::setInterval(uint8_t _dt){
	dt = _dt / 1000.0f;
}

/* PIDを動かす、一定間隔で呼び出す */
void PID::calculatePID(){
	if (locked) return;
	if (!effective) return;

	float error = (float)target - (float)enc->getAngle();
	int16_t proportional = error * kp;

	if (((int16_t)error ^ 2) < (allowError ^ 2)) error = 0.0f;

	float derivative = kd * (error - last_error) / dt;
	integral = integral + error * dt * ki;
	last_error = error;

	if (integral > max_integral) integral = max_integral;
	if (integral < -max_integral) integral = -max_integral;

	float output = proportional + integral + derivative;

	if (output > max_output) output = max_output;
	if (output < -max_output) output = -max_output;

	act->move((int16_t)output);
}

/* PIDを有効化 */
void PID::enable(){
	effective = true;
}

/* PIDを無効化 Actに直接数値を入れられる */
void PID::disable(){
	effective = false;
}

/* 積分、微分をリセット */
void PID::reset(){
	integral = 0.0f;
	last_error = 0.0f;
}

/* ロック */
void PID::lock(){
	integral = 0.0f;
	last_error = 0.0f;
	act->lock();
	locked = true;
}

/* アンロック */
void PID::unlock(){
	act->unlock();
	locked = false;
}
