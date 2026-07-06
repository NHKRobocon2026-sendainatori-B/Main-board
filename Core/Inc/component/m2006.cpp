/*
 * m2006.cpp
 *
 *  Created on: Jul 2, 2026
 *      Author: nika-
 */

#include "component/m2006.h"

/*　m2006　*/
m2006::m2006()
: targetCurrent(0), targetSpeed(0), maxSpeed(0), speed(0), ampere(0),
  temp(0), totalAngle(0), lastAngle(0), kp(0.1f), ki(0.1f), kd(0.1f),
  max_integral(0.0f), integral(0.0f), last_error(0.0f), dt(0.01f), locked(false), startFlag(true)
{
	// TODO Auto-generated constructor stub

}

m2006::~m2006() {
	// TODO Auto-generated destructor stub
}

/* 目標スピードを設定 */
void m2006::move(int16_t out){
	targetSpeed = out;
	PID_reset();
}

/* 今の角度を取得 */
int32_t m2006::getAngle(){
	return totalAngle;
}

/* 今の角度を0にする */
void m2006::setZero(){
	totalAngle = 0;
}

/* PIDの値を設定 */
void m2006::setPID(float _kp, float _ki, float _kd){
	kp = _kp;
	ki = _ki;
	kd = _kd;
}

/* 積分の最大値を設定 */
void m2006::setMaxIntegral(float _max_integral){
	max_integral = _max_integral;
}

/*　PIDの間隔を設定　*/
void m2006::setInterval(uint8_t _dt){
	dt = _dt;
}

/* PIDの計算　一定間隔で呼び出す */
void m2006::calculatePID(){
	if (locked) return;

	float error = targetSpeed - speed;
	int16_t proportional = error * kp;

	float derivative = kd * (error - last_error) / dt;
	integral = integral + error * dt * ki;
	last_error = error;

	if (integral > max_integral) integral = max_integral;
	if (integral < -max_integral) integral = -max_integral;

	float output = proportional + integral + derivative;

	int32_t max_current = 16000;
	if (output > max_current) output = max_current;
	if (output < -max_current) output = -max_current;

	int16_t current = (int16_t)output;

	targetCurrent = current;
}

/* PIDの微分、積分をリセット */
void m2006::PID_reset(){
	last_error = 0.0f;
	integral = 0.0f;
}

/* CANからのデータから取得 managerから呼び出してもらう */
void m2006::updateFromCAN(uint8_t data[8]){
	int16_t rawAngle = (int16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
	speed          	 = (int16_t)(((uint16_t)data[2] << 8) | (uint16_t)data[3]);
	ampere           = (int16_t)(((uint16_t)data[4] << 8) | (uint16_t)data[5]);
	temp             = data[6];

	if (!startFlag) {
	    int16_t diff = rawAngle - lastAngle;

	    if (diff > 4096)       diff -= 8192; //急な変化はまたいだということ
	    else if (diff < -4096) diff += 8192;

	    totalAngle += diff;
	} else {
		//最初のデータを基準にする
		startFlag = false;
	}
	lastAngle = rawAngle;
}

/* ロック */
void m2006::lock(){
	targetSpeed = 0;
	targetCurrent = 0;
	integral = 0.0f;
	locked = true;
}

/* アンロック */
void m2006::unlock(){
	locked = false;
}
