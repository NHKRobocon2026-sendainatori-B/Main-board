/*
 * PositionPIDController.cpp
 *
 *  Created on: Jul 31, 2026
 *      Author: nika-
 */

#include <PositionPIDController.h>

PositionPIDController::PositionPIDController(SpeedPIDController* speed_pid_)
: speed_pid_(speed_pid_), target(0), target_speed(0), allowError(10), max_speed(4096),
  locked(false), effective(true), out(0)
{
	// TODO Auto-generated constructor stub

}

PositionPIDController::~PositionPIDController() {
	// TODO Auto-generated destructor stub
}

/* 目標値を設定、このスピードになるようにする */
void PositionPIDController::setTarget(int32_t _target){
	target = _target;
	reset();
}

/* pidの値を設定 */
void PositionPIDController::setPID(float _kp, float _ki,float _kd){
	gain.kp = _kp;
	gain.ki = _ki;
	gain.kd = _kd;
}

/* pidの値を設定 */
void PositionPIDController::setPID(PIDgain _gain){
	gain = _gain;
}

/* 許容誤差を設定 */
void PositionPIDController::setAllowError(int16_t _allowError){
	allowError = _allowError;
}

void PositionPIDController::setMaxIntegral(float _max_integral){
	max_integral = _max_integral;
}

/* 最大出力を設定 */
void PositionPIDController::setMaxSpeed(int16_t _max_speed){
	max_speed = _max_speed;
}

/* calculatePIDが呼び出される間隔を設定、単位はms
 * speed_pidも共有される*/
void PositionPIDController::setInterval(float _dt){
	dt = _dt / 1000.0f;
}

/* 許す一秒間の変化の大きさを設定 */
void PositionPIDController::setMaxAcceleration(int32_t accele){
	if (accele < 1) return;
	max_acceleration = accele;
}

/* PIDを動かす、一定間隔で呼び出す, speed_pidもアップデート*/
void PositionPIDController::update(){
	if (locked) return;
	if (effective){
		int32_t now_angle = speed_pid_->enc->getAngle();
		float error = (float)(target - now_angle);

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

			derivative = gain.kd * (error - last_error) / dt;
		}

		last_error = error;

		if (integral > max_integral) integral = max_integral;
		if (integral < -max_integral) integral = -max_integral;

		float speed = proportional + integral * gain.ki + derivative;

		if (speed > max_speed) speed = max_speed;
		if (speed < -max_speed) speed = -max_speed;

		float max_speed_change = (float)max_acceleration * dt;

		float speed_diff = speed - target_speed;

		if (speed_diff > max_speed_change) {
			target_speed += max_speed_change;
		} else if (speed_diff < -max_speed_change) {
			target_speed -= max_speed_change;
		} else {
			target_speed = speed;
		}

		speed_pid_->setTarget(target_speed);
	}
}

/* 積分、微分をリセット */
void PositionPIDController::reset(){
	target_speed = 0.0f;
	integral = 0.0f;

	if (speed_pid_ != nullptr && speed_pid_->enc != nullptr) {
		last_error = (float)(target - speed_pid_->enc->getAngle());
	} else {
		last_error = 0.0f;
	}

	speed_pid_->reset();
}

/* PIDを有効化 */
void PositionPIDController::enable(){
	effective = true;
}

/* PIDを無効化 speedPIDに直接数値を入れられる */
void PositionPIDController::disable(){
	effective = false;
}

/* ロック */
void PositionPIDController::lock(){
	speed_pid_->lock();
	locked = true;
}

/* アンロック */
void PositionPIDController::unlock(){
	speed_pid_->unlock();
	reset();
	locked = false;
}
