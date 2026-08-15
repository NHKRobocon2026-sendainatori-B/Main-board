/*
 * m2006.cpp
 *
 *  Created on: Jul 2, 2026
 *      Author: nika-
 */

#include "component/m2006.h"

/*　m2006　*/
m2006::m2006()
{
	// TODO Auto-generated constructor stub

}

m2006::~m2006() {
	// TODO Auto-generated destructor stub
}

/* 目標出力の設定
 * 的確なスピードを変更したい場合はSpeedPIDControllerを使う */
void m2006::move(int16_t _out){
	if (locked) return;
	targetCurrent = _out;
}

/* 今の角度を取得 */
int32_t m2006::getAngle(){
	return totalAngle;
}

/* 今の角度を0にする */
void m2006::setZero(){
	totalAngle = 0;
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
		lastAngle = rawAngle;
		startFlag = false;
	}
	lastAngle = rawAngle;
}

/* ロック */
void m2006::lock(){
	locked = true;
}

/* アンロック */
void m2006::unlock(){
	locked = false;
}
