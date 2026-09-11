/*
 * Steering.cpp
 *
 *  Created on: Jul 4, 2026
 *      Author: nika-
 */

#include "Steering.h"

#define PULSEONE 1179648 //一回転のパルス
#define DRIVEMAXOUT 40 //ドライブの最大出力
//計算用配列、左前y、右前y、左後y、右後ろy、左前X、右前X、左後X、右後ろX
const float SIGNARRAY[8] = {-1, 1, -1, 1, 1, 1, -1, -1};
//タイヤと中心の距離
const float DX = 0.415; //前後
const float DY = 0.395; //左右
//向き調整用配列, 左前, 右前, 左後, 右後, steer・driveの順
const int DIRALIGN[8] = {-1, 1, -1, 1, -1, -1, -1, -1};

Steering::Steering(std::array<UnitSteering*, 4>* _units, std::array<uint16_t, 4>* interrupts)
: units(_units), interrupts(interrupts)
{
	// TODO Auto-generated constructor stub
	for(size_t i = 0; i < 4; i++){
		pending_promises_[(*interrupts)[i]].steer = (*units)[i];
	}
}

Steering::~Steering() {
	// TODO Auto-generated destructor stub
}

void Steering::init(){
	for (auto unit : *units){
		unit->init();
	}
}

//入る値はパーセント、-1~1, ros2側のxy軸
void Steering::move(float x, float y, float yaw){
	if (locked) return;
	if (shooter) return;
	if (x == 0.0f && y == 0.0f && yaw == 0.0f){
		//何も押されていないときは元の位置まで戻す
		for (auto unit : *units){
			unit->move(0, 0);
		}
		return;
	}
	//計算をここに
	for (size_t i = 0; i < units->size(); i++){
		float wheelX = x + DX * yaw * SIGNARRAY[i];
		float wheelY = y + DY * yaw * SIGNARRAY[i + 4];

		int32_t steer_val = static_cast<int32_t>(std::atan2(wheelX, wheelY) / (2 * 3.141592) * PULSEONE) * DIRALIGN[i * 2];
		int16_t drive_val = static_cast<int16_t>(std::hypot(wheelY, wheelX) * DRIVEMAXOUT) * DIRALIGN[i * 2 + 1];

		if (std::abs(steer_val - (*units)[i]->getAngle()) > (PULSEONE / 4)){
			int32_t other_angle = (steer_val > 0) ? steer_val - PULSEONE / 2 : steer_val + PULSEONE / 2;
			if (std::abs(steer_val - (*units)[i]->getAngle()) > std::abs(other_angle - (*units)[i]->getAngle())){
				steer_val = other_angle;
				drive_val *= -1;
			}
		}

		(*units)[i]->move(drive_val, steer_val);
	}
}

/*原点どり開始*/
void Steering::setZero(){
	if (shooter) return;
	locked = true;
	settingZero = true;
	for(size_t i = 0; i < 4; i++){
		(*units)[i]->setZero();
		pending_promises_[(*interrupts)[i]].promise.reset();
		futures[i] = pending_promises_[(*interrupts)[i]].promise.get_future();
	}
}

/*フォトインタラプタの割り込み、ピンとポートを引数に
 *返す値を参考に送信するプログラムを */
ProcessStatus Steering::interruptsetZero(uint16_t key){
	if (!settingZero || !locked) return ProcessStatus::IN_PROGRESS;
	auto it = pending_promises_.find(key);
    if (it == pending_promises_.end()) return ProcessStatus::IN_PROGRESS;

    it->second.steer->InterruptZero();
    it->second.promise.set_success(true);

    return setZerocheckStatus();
}

/* 一定時間ごとに呼び出す、エンコーダのデータをもとに逆に動かすか指定 */
ProcessStatus Steering::setZeroupdate(){
	if (!locked || !settingZero) return ProcessStatus::IN_PROGRESS;

	for (size_t i = 0; i < (*units).size(); i++){
		UnitSteering* unit = (*units)[i];
		if (!unit->isSettingZero()) continue;

		int32_t current_angle = unit->getSteerPID()->speed_pid_->enc->getAngle();
		int32_t diff = current_angle - unit->getFirstAngle();

		if (unit->getZeromode() == setZeroMode::ROTATE180) {
			if (diff >= PULSEONE / 2) {
				unit->Change_direction();
		    }
		} else if (unit->getZeromode() == setZeroMode::ROTATE360) {
			if (diff <= -PULSEONE / 2){
				unit->Change_direction();

				uint16_t key = (*interrupts)[i];
				auto it = pending_promises_.find(key);
				if (it != pending_promises_.end()) {
					it->second.promise.set_failed(false);
				}
			}
		}
	}
	return setZerocheckStatus();
}

/* 全ユニットの Future 状況を判定する共通処理 */
ProcessStatus Steering::setZerocheckStatus() {
    bool all_completed = std::all_of(futures.begin(), futures.end(), [](const Future<bool>& f) {
        return f.is_ready();
    });
    bool any_failed = std::any_of(futures.begin(), futures.end(), [](const Future<bool>& f) {
    	return f.is_failed();
    });

    if (all_completed) {
        bool all_success = std::all_of(futures.begin(), futures.end(), [](const Future<bool>& f){
            return f.is_success();
        });

        settingZero = false;

        if (all_success){
            locked = false;
            return ProcessStatus::SUCCESS;
        }
    } else if (any_failed) {
    	lock();
    	settingZero = false;
    	return ProcessStatus::FAILED;
    }
    return ProcessStatus::IN_PROGRESS;
}

void Steering::updateSpeed(){
	for (auto unit : *units){
		unit->getSteerPID()->speed_pid_->update();
	}
}

void Steering::updatePosition(){
	for (auto unit : *units){
		unit->getSteerPID()->update();
	}
}

void Steering::shooterMode(bool mode) {
	shooter = mode;
	if (mode) {
		(*units)[0]->move(0, 0);
		(*units)[1]->move(0, PULSEONE / 4);
		(*units)[2]->move(0, PULSEONE / 4);
		(*units)[3]->move(0, 0);
	}
}

std::array<int32_t, 4> Steering::sendSteeringAngle() {
	auto list = std::array<int32_t, 4>();
	for (size_t i = 0; i < 4; i++) {
		list[i] = (*units)[i]->getAngle() * DIRALIGN[i * 2];
	}
	return list;
}

void Steering::lock(){
	locked = true;
	for (auto unit : *units){
		unit->lock();
	}
}

void Steering::unlock(){
	locked = false;
	for (auto unit : *units){
		unit->unlock();
	}
}
