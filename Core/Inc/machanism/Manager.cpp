/*
 * Manager.cpp
 *
 *  Created on: Jul 4, 2026
 *      Author: nika-
 */

#include "Manager.h"

Manager::Manager(Loader* loader, Odometry* odometry, Shooter* shooter, Steering* steering)
: loader_(loader), odometry_(odometry), shooter_(shooter), steering_(steering)
{
	// TODO Auto-generated constructor stub

}

Manager::~Manager() {
	// TODO Auto-generated destructor stub
}

/* UARTからの割り込みに入れる, データを受け取りいろいろな場所に分配 */
void Manager::updatefromUART(uint8_t data){

}

/* タイマー割り込みに入れる, オドメトリデータを取得送信 */
void Manager::updateOdometry(){

}

/* ロック */
void Manager::lock(){
	locked = true;
	loader_->lock();
	shooter_->lock();
	steering_->lock();
}

/* アンロック */
void Manager::unlock(){
	locked = false;
	loader_->unlock();
	shooter_->unlock();
	steering_->unlock();
}
