/*
 * ESC.cpp
 *
 *  Created on: Jul 11, 2026
 *      Author: nika-
 */

#include <ESC.h>

/*ESCのプログラム、この後絶対setMaxを呼び出して
 * PWMが1増えると1μ秒かわるように調整必要*/
ESC::ESC(TIM_HandleTypeDef* _tim_handle, uint16_t _tim_channel)
: tim_handle(_tim_handle), tim_channel(_tim_channel), maxOut(0), locked(true)
{
	// TODO Auto-generated constructor stub
	HAL_TIM_PWM_Start(tim_handle, tim_channel);
	__HAL_TIM_MOE_ENABLE(tim_handle);
	__HAL_TIM_SET_COMPARE(tim_handle, tim_channel, 0);
}

ESC::~ESC() {
	// TODO Auto-generated destructor stub
}

/*最大出力を設定(1msからどのくらい変えるか、PWMの数値分追加)*/
void ESC::setMax(uint16_t _maxOut){
	maxOut = _maxOut;
	__HAL_TIM_SET_COMPARE(tim_handle, tim_channel, 1000);
	HAL_Delay(2000);
	__HAL_TIM_SET_COMPARE(tim_handle, tim_channel, 0);
	locked = false;
}

/*出力、設定したマックス値の何パーセント(0~100)なのかを入れる*/
void ESC::move(uint8_t ratio){
	if (locked) return;
	if (ratio < 0 || ratio > 100){
		ratio = 0;
	}
	__HAL_TIM_SET_COMPARE(tim_handle, tim_channel, (uint16_t)((static_cast<float>(ratio) / 100.0f) * maxOut + 1000));
}

void ESC::lock(){
	__HAL_TIM_SET_COMPARE(tim_handle, tim_channel, 0);
	locked = true;
}

void ESC::unlock(){
	locked = false;
}
