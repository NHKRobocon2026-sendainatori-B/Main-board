/*
 * Servo.cpp
 *
 *  Created on: Jul 11, 2026
 *      Author: nika-
 */

#include <Servo.h>

/*サーボのプログラム、これの後絶対settingを呼び出して*/
Servo::Servo(TIM_HandleTypeDef* _tim_handle, uint16_t _tim_channel)
: tim_handle(_tim_handle), tim_channel(_tim_channel), locked(true)
{
	// TODO Auto-generated constructor stub
	HAL_TIM_PWM_Start(tim_handle, tim_channel);
	__HAL_TIM_MOE_ENABLE(tim_handle);
	__HAL_TIM_SET_COMPARE(tim_handle, tim_channel, 0);
}

Servo::~Servo() {
	// TODO Auto-generated destructor stub
}

/*出力する、60文法の値をいれる(0から180度)*/
void Servo::move(uint16_t angle){
	if (locked) return;
	if (angle > 180) {
		angle = 180;
	}
	uint16_t compare = (uint16_t)((static_cast<float>(angle) / 180.0f) * (out180 - out0)) + out0;
	__HAL_TIM_SET_COMPARE(tim_handle, tim_channel, compare);
}

/*0度と180度の場合の出力を設定、これを基準にする*/
void Servo::setting(uint16_t _out0, uint16_t _out180){
	out0 = _out0;
	out180 = _out180;
	locked = false;
}

void Servo::lock(){
	locked = true;
}

void Servo::unlock(){
	locked = false;
}
