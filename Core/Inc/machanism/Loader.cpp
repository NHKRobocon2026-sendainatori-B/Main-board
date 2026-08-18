/*
 * Loader.cpp
 *
 *  Created on: Jul 4, 2026
 *      Author: nika-
 */

#include "Loader.h"

#define SHOOTERUPPERSECONDS 100 //上に少し動かすときのミリ秒
#define SHOOTERUPPEROUT 100 //上に少し動かすときの出力
#define SHOOTERARMOUT 200 //射出に装填するモーターを動かすPWM
#define SHOOTERARMSECONDS 200 //射出に装填するモーターを動かす時間

#define SHOOTERUPPERADDRESS 0
#define SHOOTERARMADDRESS 1
#define LOADDOWNERADDRESS 2
#define LOADSERVOUPPADDRESS 3
#define LOADMOTOROUTADDRESS 4
#define LOADSERVODOWNERADDRESS 5
#define LOADMOTORINADDRESS 6
#define LOADUPPERADDRESS 7

#define M2006KP 0.1
#define M2006KI 0.0001
#define M2006KD 0.01
#define M2006INTERVAL 10
#define M2006ALLOWERROR 100
#define M2006MAXOUTPUT 2000
#define M2006PULSE 8192

Loader::Loader(Servo* shovel_, MD4ch_child* arm_, MD4ch_child* elevator_, SpeedPIDController* importer_upper_, SpeedPIDController* importer_below_)
: shovel_(shovel_), arm_(arm_), elevator_(elevator_), importer_upper_(importer_upper_), importer_below_(importer_below_)
{
	// TODO Auto-generated constructor stub
	promises[SHOOTERUPPERADDRESS] = Promise<bool>(); //射出装填, 上にすこしずつ
	promises[SHOOTERARMADDRESS] = Promise<bool>(); //射出装填, 交互にモーターを回す
	promises[LOADDOWNERADDRESS] = Promise<bool>(); //弾丸装填, 下に移動
	promises[LOADSERVOUPPADDRESS] = Promise<bool>(); //弾丸装填, サーボ上に
	promises[LOADMOTOROUTADDRESS] = Promise<bool>(); //弾丸装填, モーターを外側に
	promises[LOADSERVODOWNERADDRESS] = Promise<bool>(); //弾丸装填, サーボを下に
	promises[LOADMOTORINADDRESS] = Promise<bool>(); //弾丸装填, モーターを内側に
	promises[LOADUPPERADDRESS] = Promise<bool>(); //弾丸装填, 上に移動
	//futureに入れる
	futures[SHOOTERUPPERADDRESS] = promises[0].get_future();
	futures[SHOOTERARMADDRESS] = promises[1].get_future();
	futures[LOADDOWNERADDRESS] = promises[2].get_future();
	futures[LOADSERVOUPPADDRESS] = promises[3].get_future();
	futures[LOADMOTOROUTADDRESS] = promises[4].get_future();
	futures[LOADSERVODOWNERADDRESS] = promises[5].get_future();
	futures[LOADMOTORINADDRESS] = promises[6].get_future();
	futures[LOADUPPERADDRESS] = promises[7].get_future();

	arm_->setMode(Mode::OPENLOOP);
	importer_upper_->setPID(M2006KP, M2006KI, M2006KD);
	importer_upper_->setInterval(M2006INTERVAL);
	importer_upper_->setAllowError(M2006ALLOWERROR);
	importer_upper_->setMaxOutput(M2006MAXOUTPUT);
	importer_upper_->setPulse(M2006PULSE);
	importer_below_->setPID(M2006KP, M2006KI, M2006KD);
	importer_below_->setInterval(M2006INTERVAL);
	importer_below_->setAllowError(M2006ALLOWERROR);
	importer_below_->setMaxOutput(M2006MAXOUTPUT);
	importer_below_->setPulse(M2006PULSE);
}

Loader::~Loader() {
	// TODO Auto-generated destructor stub
}

/*shooterを動かすかを変更、もし机から雑巾を格納中ならfalseを返す*/
bool Loader::shooterMove(bool move){
	if (state == State::IBLE || state == State::SHOOTER_MOVE){
		state = (move) ? State::SHOOTER_MOVE : State::IBLE;
		return true;
	} else {
		return false;
	}
}

/*机からの装填*/
bool Loader::loadbullet(){
	if (state == State::SHOOTER_MOVE) return false;
	state = State::DESK_LOAD;
	flag_counters[SHOOTERARMADDRESS] = ms_counter;
	promises[SHOOTERARMADDRESS].reset();
	return true;
}

/* フォトインタラプタの割り込みが来る, shooterからフォトインタラプタの割り込み一定回ごとに呼び出して */
void Loader::shooterInterrupt(bool elevate, bool direction){
	if (state != State::SHOOTER_MOVE) return;
	if (elevate) {
		arm_->setOut(SHOOTERUPPEROUT);
		promises[SHOOTERUPPERADDRESS].reset();
		flag_counters[SHOOTERUPPERADDRESS] = ms_counter;
	} else {
		int16_t out = (direction) ? SHOOTERLOADOUT : -SHOOTERLOADOUT;
		arm_->setOut(out);
		promises[SHOOTERARMADDRESS].reset();
		flag_counters[SHOOTERARMADDRESS] = ms_counter;
	}
}

/* 射出装填用のアップデート, 10msごとに更新 */
void Loader::shooterUpdate(){
	if (state != State::SHOOTER_MOVE) return;
	ms_counter += 10;
	if (!futures[SHOOTERARMADDRESS].is_ready() && (ms_counter - flag_counters[SHOOTERARMADDRESS]) > SHOOTERARMSECONDS){
		arm_->setOut(0);
		promises[SHOOTERARMADDRESS].set_value(true);
	}
	if (!futures[SHOOTERUPPERADDRESS].is_ready() && (ms_counter - flag_counters[SHOOTERUPPERADDRESS]) > SHOOTERUPPERSECONDS) {
		elevator_->setOut(0);
		promises[SHOOTERUPPERADDRESS].set_value(true);
	}
}

/* 机から装填のアップデート, 10msごとに更新 */
void Loader::bulletUpdate(){
	if (state != State::DESK_LOAD) return;
	ms_counter += 10;

}

void Loader::lock(){
	locked = true;
}

void Loader::unlock(){
	locked = false;
}
