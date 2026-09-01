/*
 * Manager.cpp
 *
 *  Created on: Jul 4, 2026
 *      Author: nika-
 */

#include "Manager.h"

Manager::Manager(Loader* loader, Odometry* odometry, Shooter* shooter, Steering* steering, UART_HandleTypeDef* huart_)
: loader_(loader), odometry_(odometry), shooter_(shooter), steering_(steering), huart_(huart_)
{
	// TODO Auto-generated constructor stub

}

Manager::~Manager() {
	// TODO Auto-generated destructor stub
}

/* UARTからの割り込みに入れる, データを受け取りいろいろな場所に分配 */
void Manager::updatefromUART(uint8_t data){
	if (state == State::HEADER){
		state = State::READ;
		switch(data){
		case 0x11:
			target = Target::SHOOTER;
			break;
		case 0x3F:
			target = Target::STEER;
			break;
		case 0x15:
			target = Target::LOADER;
			break;
		case 0xFF:
			target = Target::LOCK;
			break;
		case 0x63:
			target = Target::START;
			break;
		case 0x26:
			target = Target::RESET;
			break;
		default:
			state = State::HEADER;
			break;
		}
	} else if (state == State::READ){
		switch(target){
		case Target::STEER:
			_readSteer(data);
			break;
		case Target::SHOOTER:
			_readShooter(data);
			break;
		case Target::LOADER:
			_readLoader(data);
			break;
		case Target::LOCK:
			_readLock(data);
			break;
		case Target::START:
			if (data == 0x73){
				state = State::FOOTER;
			} else {
				state = State::HEADER;
			}
			break;
		case Target::RESET:
			if (data == 0x89){
				state = State::FOOTER;
			} else {
				state = State::HEADER;
			}
			break;
		}
	} else if (state == State::FOOTER){
		state = State::HEADER;
		switch(target){
		case Target::STEER:
			if (data == 0x4E){
				steering_->move(
					static_cast<float>(static_cast<int8_t>(logger_[0])) / 100.0f,
					static_cast<float>(static_cast<int8_t>(logger_[1])) / 100.0f,
					static_cast<float>(static_cast<int8_t>(logger_[2])) / 100.0f
				);
			}
			break;
		case Target::SHOOTER:
			if (data == 0x96){
				//shooterを回す
				//結果をUARTで送信
				_responceShooter(true); //この中にうごかすプログラム
			}
			break;
		case Target::LOADER:
			if (data == 0xF1){
				//loader動かす
				//結果をUARTで送信
				_responceLoader(true); //この中にうごかすプログラム
			}
			break;
		case Target::LOCK:
			if (data == 0x29){
				//全てにロックかアンロック
				//結果をUARTで送信
				if (logger_[0] == 0x42){
					_responceLock(lock());
				} else if (logger_[0] == 0xBD){
					_responceLock(unlock());
				} else {
					_responceLock(false);
				}
			}
			break;
		case Target::START:
			if (data == 0x03){
				//スタートさせる処理
				if (steering_ != nullptr) {
					steering_->unlock();
					steering_->setZero();
				}
				if (shooter_ != nullptr) shooter_->unlock();
				if (loader_ != nullptr) loader_->unlock();
			}
			break;
		case Target::RESET:
			if (data == 0x6F) {
				if (locked) {
					//フラグを立てる
					resetflag = true;
				}
			}
		}
		logger_.clear();
	}
}

/* ゼロ点合わせの成功失敗を送る
 * ステアリングのsetzeroupdateの結果をもとに */
void Manager::sendSetzero(bool success){
	uint8_t message[3];

	message[0] = 0x23;
	message[1] = (success) ? 0x38 : 0xF3;
	message[2] = 0x91;

	HAL_UART_Transmit(huart_, message, 3, 10);
}

/* タイマー割り込みに入れる, オドメトリデータを取得送信 */
void Manager::updateOdometry(){
	//odometryからデータを取得
	auto data = std::array<int32_t, 10>();

	uint8_t message[4];
	message[0] = 0x5E;
	message[1] = static_cast<uint8_t>((data[0] >> 24) & 0xFF);
	message[2] = static_cast<uint8_t>((data[0] >> 16) & 0xFF);
	message[3] = static_cast<uint8_t>((data[0] >> 8) & 0xFF);
	message[4] = static_cast<uint8_t>(data[0] & 0xFF);
	message[5] = static_cast<uint8_t>((data[1] >> 24) & 0xFF);
	message[6] = static_cast<uint8_t>((data[1] >> 16) & 0xFF);
	message[7] = static_cast<uint8_t>((data[1] >> 8) & 0xFF);
	message[8] = static_cast<uint8_t>(data[1] & 0xFF);
	message[9] = 0x38;

	HAL_UART_Transmit(huart_, message, 10, 10);
}

/* ロック */
bool Manager::lock(){
	locked = true;
	if (loader_ != nullptr) loader_->lock();
	if (shooter_ != nullptr) shooter_->lock();
	if (steering_ != nullptr) steering_->lock();
	return true;
}

/* アンロック */
bool Manager::unlock(){
	locked = false;
	if (loader_ != nullptr) loader_->unlock();
	if (shooter_ != nullptr) shooter_->unlock();
	if (steering_ != nullptr) steering_->unlock();
	if (resetflag){
		//ゼロ点合わせ等緊急停止ボタンを押した後しなければならないこと
		if (steering_ != nullptr) steering_->setZero();
		resetflag = false;
	}
	return true;
}

void Manager::_readSteer(uint8_t data){
	logger_.push_back(data);
	if (logger_.size() == 3) state = State::FOOTER;
}

void Manager::_readLoader(uint8_t data){
	state = (data == 0x4D) ? State::FOOTER : State::HEADER;
}

void Manager::_readShooter(uint8_t data){
	if (data != 0x22 && data != 0xD1){
		state = State::HEADER;
		return;
	}
	state = State::FOOTER;
	logger_.push_back(data);
}

void Manager::_readLock(uint8_t data){
	if (data != 0x42 && data != 0xBD){
		state = State::HEADER;
		return;
	}
	state = State::FOOTER;
	logger_.push_back(data);
}

void Manager::_responceShooter(bool success){
	uint8_t message[4];

	message[0] = 0xAA;
	message[1] = 0x11;
	message[2] = (success) ? 0x01 : 0xFE;
	message[3] = 0x55;

	HAL_UART_Transmit(huart_, message, 4, 10);
}

void Manager::_responceLoader(bool success){
	uint8_t message[4];

	message[0] = 0xAA;
	message[1] = 0x15;
	message[2] = (success) ? 0x01 : 0xFE;
	message[3] = 0x55;

	HAL_UART_Transmit(huart_, message, 4, 10);
}

void Manager::_responceLock(bool success){
	uint8_t message[4];

	message[0] = 0xAA;
	message[1] = 0xFF;
	message[2] = (success) ? 0x01 : 0xFE;
	message[3] = 0x55;

	HAL_UART_Transmit(huart_, message, 4, 10);
}
