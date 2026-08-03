/*
 * SpeedPIDController.cpp
 *
 *  Created on: Jul 31, 2026
 *      Author: nika-
 */

#include <SpeedPIDController.h>

//speed(速度制御)を扱う
SpeedPIDController::SpeedPIDController(Actuator* _act, Encoder* _enc)
: act(_act), enc(_enc), target(0),
  max_integral(10.0f), integral(0.0f), last_error(0.0f), dt(0.01f), allowError(10), max_output(4096),
  locked(false), effective(true), out(0)
{
	// TODO Auto-generated constructor stub

}

SpeedPIDController::~SpeedPIDController() {
	// TODO Auto-generated destructor stub
}

/* 目標値を設定、このスピードになるようにする */
void SpeedPIDController::setTarget(int16_t _target){
	target = _target;
	last_angle = enc->getAngle();
}

/* pidの値を設定 */
void SpeedPIDController::setPID(float _kp, float _ki, float _kd){
	gain.kp = _kp;
	gain.ki = _ki;
	gain.kd = _kd;
}

void SpeedPIDController::setPID(PIDgain _gain){
	gain.kp = _gain.kp;
	gain.ki = _gain.ki;
	gain.kd = _gain.kd;
}

/* 許容誤差を設定 */
void SpeedPIDController::setAllowError(int16_t _allowError){
	allowError = _allowError;
}

/* 積分の最大値を設定 */
void SpeedPIDController::setMaxIntegral(float _max_integral){
	max_integral = _max_integral;
}

/* 最大出力を設定 */
void SpeedPIDController::setMaxOutput(int16_t _max_output){
	max_output = _max_output;
}

/* calculatePIDが呼び出される間隔を設定、単位はms */
void SpeedPIDController::setInterval(float _dt){
	dt = _dt / 1000.0f;
}

/* PIDを動かす、一定間隔で呼び出す */
void SpeedPIDController::update(){
	if (locked) return;
	if (!effective) return;

	int32_t now_angle = enc->getAngle();
	float now_speed = (float)(now_angle - last_angle) / dt;
	float error = target - now_speed;
	float proportional = error * gain.kp;

	if ((int16_t)(error * error) < (allowError * allowError)) error = 0.0f;

	float derivative = gain.kd * (error - last_error) / dt;
	integral += error * dt;
	last_error = error;

	if (integral > max_integral) integral = max_integral;
	if (integral < -max_integral) integral = -max_integral;

	float output = proportional + integral * gain.ki + derivative;

	if (output > max_output) output = max_output;
	if (output < -max_output) output = -max_output;

	act->move((int16_t)output);
	out = (int16_t)output;

	last_angle = now_angle;
}

/* PIDを有効化 */
void SpeedPIDController::enable(){
	effective = true;
}

/* PIDを無効化 Actに直接数値を入れられる */
void SpeedPIDController::disable(){
	effective = false;
}

/* 積分、微分をリセット */
void SpeedPIDController::reset(){
	integral = 0.0f;
	last_error = 0.0f;
}

/* ロック */
void SpeedPIDController::lock(){
	integral = 0.0f;
	last_error = 0.0f;
	act->lock();
	locked = true;
}

/* アンロック */
void SpeedPIDController::unlock(){
	act->unlock();
	last_angle = enc->getAngle();
	locked = false;
}

