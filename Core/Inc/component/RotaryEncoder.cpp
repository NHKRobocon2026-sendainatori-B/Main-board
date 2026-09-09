/*
 * RotaryEncoder.cpp
 *
 *  Created on: Jul 4, 2026
 *      Author: nika-
 */

#include "component/RotaryEncoder.h"

/* ロータリーエンコーダ */
RotaryEncoder::RotaryEncoder(TIM_HandleTypeDef* _tim_handle)
: tim_handle(_tim_handle)
{
	// TODO Auto-generated constructor stub
#if defined(TIM2) && defined(TIM5)
    if (tim_handle->Instance == TIM2 || tim_handle->Instance == TIM5) {
        is_32bit = true;
    }
#elif defined(TIM2)
    if (tim_handle->Instance == TIM2) {
        is_32bit = true;
    }
#endif
    start();
}

RotaryEncoder::~RotaryEncoder() {
	// TODO Auto-generated destructor stub
}

/* 計測開始 */
void RotaryEncoder::start(){
	HAL_TIM_Encoder_Start(tim_handle, TIM_CHANNEL_ALL);
	last_counter_value = __HAL_TIM_GET_COUNTER(tim_handle);
	total_count = 0;
}

/* 計測終了 */
void RotaryEncoder::stop(){
	HAL_TIM_Encoder_Stop(tim_handle, TIM_CHANNEL_ALL);
}

/*　アップデート、一定間隔で呼び出したら確実では？　*/
void RotaryEncoder::updateAngle(){
	uint32_t current_counter = __HAL_TIM_GET_COUNTER(tim_handle);
	int32_t diff = 0;

	if (is_32bit) {
		diff = (int32_t)(current_counter - last_counter_value);
	} else {
		diff = (int16_t)(current_counter - last_counter_value);
	}

	total_count += diff;
	last_counter_value = current_counter;
}

/* 今の角度を取得 */
int32_t RotaryEncoder::getAngle(){
	updateAngle();
	return total_count;
}

/* 今の位置を0にする */
void RotaryEncoder::setZero(){
	last_counter_value = __HAL_TIM_GET_COUNTER(tim_handle);
	total_count = 0;
}
