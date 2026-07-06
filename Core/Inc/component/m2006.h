/*
 * m2006.h
 *
 *  Created on: Jul 2, 2026
 *      Author: nika-
 */

#ifndef INC_COMPONENT_M2006_H_
#define INC_COMPONENT_M2006_H_

#include <stddef.h>

#include <main.h>

#include "component/private/Actuator.h"
#include "component/private/Encoder.h"

class m2006 : public Actuator, public Encoder {
public:
	m2006();
	virtual ~m2006();

	void move(int16_t _out) override;
	int32_t getAngle() override;
	void setZero() override;

	void setPID(float _kp, float _ki, float _kd);
	void setMaxIntegral(float _max_integral);
	void calculatePID();
	void PID_reset();
	void setInterval(uint8_t _dt);
	void updateFromCAN(uint8_t data[8]);

	int16_t getTargetCurrent() {return targetCurrent;};
	int16_t getSpeed() {return speed;};
	int16_t getAmpere() {return ampere;};
	int8_t getTemp() {return temp;};

	void lock() override;
	void unlock() override;

private:
	int16_t targetCurrent; //出力
	int16_t targetSpeed; // 目標速度を保持する変数
	int16_t maxSpeed; //出せる最大速度、負の値はこれを負にする
	int16_t speed; // 現在の回転速度(rpm)
	int16_t ampere; //電流を入れる
	int8_t temp; //温度を入れる
	int32_t totalAngle;
	int16_t lastAngle; //前回の角度を格納

	/*PID関係*/
	float kp, ki, kd;
	float max_integral;
	float integral;
	float last_error;
	float dt;

	bool locked;
	bool startFlag;
};

#endif /* INC_COMPONENT_M2006_H_ */
