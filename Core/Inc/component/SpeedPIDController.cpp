/*
 * SpeedPIDController.cpp
 *
 *  Created on: Jul 31, 2026
 *      Author: nika-
 */

#include <SpeedPIDController.h>

//speed(速度制御)を扱う
SpeedPIDController::SpeedPIDController(Actuator* _act, Encoder* _enc)
: act(_act), enc(_enc)
{
	// TODO Auto-generated constructor stub
	last_angle = enc->getAngle();
}

SpeedPIDController::~SpeedPIDController() {
	// TODO Auto-generated destructor stub
}

/* 目標値を設定、このスピードになるようにする(rpm) */
void SpeedPIDController::setTarget(int16_t _target){
	target = _target;
}

/* pidの値を設定 */
void SpeedPIDController::setPID(float _kp, float _ki, float _kd){
	gain.kp = _kp;
	gain.ki = _ki;
	gain.kd = _kd;
}

void SpeedPIDController::setPulse(float _pulse){
	pulse = _pulse;
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
	float pulse_per_sec = static_cast<float>(now_angle - last_angle) / dt;

	float now_rpm = (pulse_per_sec / pulse) * 60.0f;
	float error = target - now_rpm;

	float derivative = 0.0f;
	float proportional = 0.0f;

	if (std::fabs(error) <= allowError){
		error = 0.0f;
		integral = 0.0f;
		derivative = 0.0f;
	} else {
		proportional = error * gain.kp;

		integral += error * dt;
		if (integral > max_integral) integral = max_integral;
		if (integral < -max_integral) integral = -max_integral;

		derivative = gain.kd * (now_rpm - last_rpm) / dt;
	}

	last_rpm = now_rpm;
	last_angle = now_angle;

	if (integral > max_integral) integral = max_integral;
	if (integral < -max_integral) integral = -max_integral;

	float output = proportional + integral * gain.ki + derivative;

	if (output > max_output) output = max_output;
	if (output < -max_output) output = -max_output;

	act->move((int16_t)output);
}

/* PIDを有効化 */
void SpeedPIDController::enable(){
	effective = true;
	last_angle = enc->getAngle();
}

/* PIDを無効化 Actに直接数値を入れられる */
void SpeedPIDController::disable(){
	effective = false;
}

/* 積分、微分をリセット */
void SpeedPIDController::reset(){
	integral = 0.0f;
	last_rpm = 0.0f;
	if (enc != nullptr) {
		last_angle = enc->getAngle();
	}
}

/* ロック */
void SpeedPIDController::lock(){
	act->lock();
	target = 0;
	locked = true;
}

/* アンロック */
void SpeedPIDController::unlock(){
	act->unlock();
	reset();
	locked = false;
}

