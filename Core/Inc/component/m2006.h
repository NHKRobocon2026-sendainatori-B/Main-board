/*
 * m2006.h
 *
 * m2006のライブラリです
 *
 * <定義例>
 * m2006 m1();
 * SpeedPIDController pid(&m1, &m1);
 * m1.initPID(&pid);
 *
 * (この下でm2006managerにアドレスを入れてください、
 * また、speedPIDControllerの更新も含め、全てmanager側で行います)
 *
 *  Created on: Jul 2, 2026
 *      Author: nika-
 */

#ifndef INC_COMPONENT_M2006_H_
#define INC_COMPONENT_M2006_H_

#include <stddef.h>

#include <main.h>

#include "private/Actuator.h"
#include "private/Encoder.h"
#include "SpeedPIDController.h"

class m2006 : public Actuator, public Encoder {
public:
	m2006();
	virtual ~m2006();

	int32_t getAngle() override;
	void setZero() override;
	void move(int16_t _out) override;

	void updateFromCAN(uint8_t data[8]);

	int16_t getTargetCurrent() {return targetCurrent;};
	int16_t getSpeed() {return speed;};
	int16_t getAmpere() {return ampere;};
	int8_t getTemp() {return temp;};

	void lock() override;
	void unlock() override;

private:

	int16_t targetCurrent = 0; //出力
	int32_t max_current = 0;
	int16_t targetSpeed = 0; // 目標速度を保持する変数
	int16_t maxSpeed = 0; //出せる最大速度、負の値はこれを負にする
	int16_t speed = 0; // 現在の回転速度(rpm)
	int16_t ampere = 0; //電流を入れる
	int8_t temp = 0; //温度を入れる
	int32_t totalAngle = 0;
	int16_t lastAngle = 0; //前回の角度を格納

	bool locked = false;
	bool startFlag = true;
};

#endif /* INC_COMPONENT_M2006_H_ */
