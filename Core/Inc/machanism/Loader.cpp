/*
 * Loader.cpp
 *
 *  Created on: Jul 4, 2026
 *      Author: nika-
 */

/*
 * arm_ : 50
 * elevator_ : 30
 * */

#include "Loader.h"

#define SHOOTERUPPERSECONDS 100 //上に少し動かすときのミリ秒
#define SHOOTERUPPEROUT 100 //上に少し動かすときの出力
#define SHOOTERARMOUT 200 //射出に装填するモーターを動かすPWM
#define SHOOTERARMSECONDS 200 //射出に装填するモーターを動かす時間
#define LOADAPPDOWNOUT 100 //上下移動の速さ
#define LOADAPPDOWNSECONDS 1000 //上下移動の時間
#define LOADMOTOROUT 100 //外側へと動くモーターの速さ
#define LOADMOTORSECONDS 500 //外側へと動くモーターの時間
#define LOADSERVOOPEN 120 //サーボの開いた角度
#define LOADSERVOCLOSE 0 //サーボの閉じた角度
#define LOADSERVOSECONDS 300 //少しだけ待つように

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
	promises[SHOOTERUPPERADDRESS] = {SHOOTERUPPERSECONDS, Promise<bool>()}; //射出装填, 上にすこしずつ
	promises[SHOOTERARMADDRESS] = {SHOOTERARMSECONDS, Promise<bool>()}; //射出装填, 交互にモーターを回す
	promises[LOADDOWNERADDRESS] = {LOADAPPDOWNSECONDS, Promise<bool>()}; //弾丸装填, 下に移動
	promises[LOADSERVOUPPADDRESS] = {LOADSERVOSECONDS, Promise<bool>()}; //弾丸装填, サーボ上に
	promises[LOADMOTOROUTADDRESS] = {LOADMOTORSECONDS, Promise<bool>()}; //弾丸装填, モーターを外側に
	promises[LOADSERVODOWNERADDRESS] = {LOADSERVOSECONDS, Promise<bool>()}; //弾丸装填, サーボを下に
	promises[LOADMOTORINADDRESS] = {LOADMOTORSECONDS, Promise<bool>()}; //弾丸装填, モーターを内側に
	promises[LOADUPPERADDRESS] = {LOADAPPDOWNSECONDS, Promise<bool>()}; //弾丸装填, 上に移動

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
	if (state == State::DESK_LOAD) return false;
	if (!move && state == State::SHOOTER_MOVE) {
		//Shooter関連のfutureをクリア
		futures[SHOOTERUPPERADDRESS].clear();
		futures[SHOOTERARMADDRESS].clear();
	}
	state = (move) ? State::SHOOTER_MOVE : State::IDLE;
	return true;
}

/*机からの装填*/
bool Loader::loadbullet(){
	if (state == State::SHOOTER_MOVE) return false;
	state = State::DESK_LOAD;
	flag_counters[LOADDOWNERADDRESS] = ms_counter;
	promises[LOADDOWNERADDRESS].promise.reset();
	futures[LOADDOWNERADDRESS] = promises[LOADDOWNERADDRESS].promise.get_future();
	elevator_->setOut(-LOADAPPDOWNOUT);
	return true;
}

/* フォトインタラプタの割り込みが来る, shooterからフォトインタラプタの割り込み一定回ごとに呼び出して */
void Loader::shooterInterrupt(bool elevate, bool direction){
	if (state != State::SHOOTER_MOVE) return;
	if (elevate) {
		arm_->setOut(SHOOTERUPPEROUT);
		promises[SHOOTERUPPERADDRESS].promise.reset();
		futures[SHOOTERUPPERADDRESS] = promises[SHOOTERUPPERADDRESS].promise.get_future();
		flag_counters[SHOOTERUPPERADDRESS] = ms_counter;
	} else {
		int16_t out = (direction) ? SHOOTERARMOUT : -SHOOTERARMOUT;
		arm_->setOut(out);
		promises[SHOOTERARMADDRESS].promise.reset();
		futures[SHOOTERARMADDRESS] = promises[SHOOTERARMADDRESS].promise.get_future();
		flag_counters[SHOOTERARMADDRESS] = ms_counter;
	}
}

/* 射出装填用のアップデート, 10msごとに更新 */
void Loader::shooterUpdate(){
	if (state != State::SHOOTER_MOVE) return;
	ms_counter += 10;
	if (futures[SHOOTERARMADDRESS].valid()){
		if (!futures[SHOOTERARMADDRESS].is_ready() && (ms_counter - flag_counters[SHOOTERARMADDRESS]) > SHOOTERARMSECONDS){
			arm_->setOut(0);
			futures[SHOOTERARMADDRESS].clear();
			promises[SHOOTERARMADDRESS].promise.set_success(true);
		}
	}
	if (futures[SHOOTERUPPERADDRESS].valid()){
		if (!futures[SHOOTERUPPERADDRESS].is_ready() && (ms_counter - flag_counters[SHOOTERUPPERADDRESS]) > SHOOTERUPPERSECONDS) {
			elevator_->setOut(0);
			futures[SHOOTERUPPERADDRESS].clear();
			promises[SHOOTERUPPERADDRESS].promise.set_success(true);
		}
	}
}

/* 机から装填のアップデート, 10msごとに更新 */
void Loader::bulletUpdate(){
	if (state != State::DESK_LOAD) return;
	ms_counter += 10;
	for (size_t id =  LOADDOWNERADDRESS; id < futures.size(); id++) {
		if (!futures[id].valid()) continue;

		if (!futures[id].is_ready() && (ms_counter - flag_counters[id]) > promises[id].time){
			//止めて次の物を動かす
			futures[id].clear();
			promises[id].promise.set_success(true);
			size_t next = id + 1;
			if (next == futures.size()){
				state = State::IDLE;
				elevator_->setOut(0);
				return;
			}
			promises[next].promise.reset();
			futures[next] = promises[next].promise.get_future();
			promises[next].time = ms_counter;
			switch(next){
			case LOADSERVOUPPADDRESS:
				elevator_->setOut(0);
				shovel_->move(LOADSERVOOPEN);
				break;
			case LOADMOTOROUTADDRESS:
				importer_upper_->setTarget(-LOADMOTOROUT);
				importer_below_->setTarget(LOADMOTOROUT);
				break;
			case LOADSERVODOWNERADDRESS:
				importer_upper_->setTarget(0);
				importer_upper_->setTarget(0);
				shovel_->move(LOADSERVOCLOSE);
				break;
			case LOADMOTORINADDRESS:
				importer_upper_->setTarget(LOADMOTOROUT);
				importer_below_->setTarget(-LOADMOTOROUT);
				break;
			case LOADUPPERADDRESS:
				importer_upper_->setTarget(0);
				importer_below_->setTarget(0);
				elevator_->setOut(LOADAPPDOWNOUT);
				break;
			}
			break;
		}
	}
}

void Loader::lock(){
	locked = true;
}

void Loader::unlock(){
	locked = false;
}
