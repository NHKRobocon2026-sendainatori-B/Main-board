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

#define SHOOTERUPPERSECONDS 1000 //上に少し動かすときのミリ秒
#define SHOOTERUPPEROUT 30 //上に少し動かすときの出力
#define SHOOTERARMOUT 30 //射出に装填するモーターを動かすPWM
#define SHOOTERARMSECONDS 1500 //射出に装填するモーターを動かす時間
#define LOADAPPDOWNOUT 100 //上下移動の速さ
#define LOADAPPDOWNSECONDS 1000 //上下移動の時間
#define LOADMOTORPOSITION 2492006 //移動する距離、＋で開く方向
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

#define SPEED_KP 0.1281f
#define SPEED_KI 0.01f
#define SPEED_KD 0.0009184f
#define SPEED_INTERVAL 5
#define SPEED_ALLOWERROR 300
#define SPEED_INTEGRAL 100
#define SPPED_MAXOUTPUT 6000
#define SPPED_PULSE 8192

#define POSITION_KP 0.018f
#define POSITION_KI 0.001f
#define POSITION_KD 0.0f
#define POSITION_INTERVAL 10
#define POSITION_ACCEL 50000
#define POSITION_INTEGRAL 100
#define POSITION_MAXSPEED 9000

Loader::Loader(Servo* shovel_, MD4ch_child* arm_, MD4ch_child* elevator_, PositionPIDController* importer_upper_, PositionPIDController* importer_below_)
: shovel_(shovel_), arm_(arm_), elevator_(elevator_), importer_upper_(importer_upper_), importer_below_(importer_below_)
{
	// TODO Auto-generated constructor stub
}

Loader::~Loader() {
	// TODO Auto-generated destructor stub
}

void Loader::init() {
	promises[SHOOTERUPPERADDRESS] = Promise<bool>(); //射出装填, 上にすこしずつ
	promises[SHOOTERARMADDRESS] = Promise<bool>(); //射出装填, 交互にモーターを回す
	promises[LOADDOWNERADDRESS] = Promise<bool>(); //弾丸装填, 下に移動
	promises[LOADSERVOUPPADDRESS] = Promise<bool>(); //弾丸装填, サーボ上に
	promises[LOADMOTOROUTADDRESS] = Promise<bool>(); //弾丸装填, モーターを外側に
	promises[LOADSERVODOWNERADDRESS] = Promise<bool>(); //弾丸装填, サーボを下に
	promises[LOADMOTORINADDRESS] = Promise<bool>(); //弾丸装填, モーターを内側に
	promises[LOADUPPERADDRESS] = Promise<bool>(); //弾丸装填, 上に移動

	shovel_->setting(1000, 2000);

	arm_->setMode(Mode::OPENLOOP);
	elevator_->setMode(Mode::OPENLOOP);

	importer_upper_->speed_pid_->setInterval(SPEED_INTERVAL);
	importer_upper_->speed_pid_->setMaxIntegral(SPEED_INTEGRAL);
	importer_upper_->speed_pid_->setMaxOutput(SPPED_MAXOUTPUT);
	importer_upper_->speed_pid_->setPID(SPEED_KP, SPEED_KI, SPEED_KD);
	importer_upper_->speed_pid_->setPulse(SPPED_PULSE);
	importer_upper_->setInterval(POSITION_INTERVAL);
	importer_upper_->setMaxAcceleration(POSITION_ACCEL);
	importer_upper_->setMaxIntegral(POSITION_INTEGRAL);
	importer_upper_->setMaxSpeed(POSITION_MAXSPEED);
	importer_upper_->setPID(POSITION_KP, POSITION_KI, POSITION_KD);

	importer_below_->speed_pid_->setInterval(SPEED_INTERVAL);
	importer_below_->speed_pid_->setMaxIntegral(SPEED_INTEGRAL);
	importer_below_->speed_pid_->setMaxOutput(SPPED_MAXOUTPUT);
	importer_below_->speed_pid_->setPID(SPEED_KP, SPEED_KI, SPEED_KD);
	importer_below_->speed_pid_->setPulse(SPPED_PULSE);
	importer_below_->setInterval(POSITION_INTERVAL);
	importer_below_->setMaxAcceleration(POSITION_ACCEL);
	importer_below_->setMaxIntegral(POSITION_INTEGRAL);
	importer_below_->setMaxSpeed(POSITION_MAXSPEED);
	importer_below_->setPID(POSITION_KP, POSITION_KI, POSITION_KD);
}

/*shooterを動かすかを変更、もし机から雑巾を格納中ならfalseを返す*/
bool Loader::shooterMove(bool move){
	if (locked) return false;
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
	if (locked) return false;
	if (state == State::SHOOTER_MOVE) return false;
	state = State::DESK_LOAD;
	flag_counters[LOADDOWNERADDRESS] = ms_counter;
	elapsed_time = LOADAPPDOWNSECONDS;
	promises[LOADDOWNERADDRESS].reset();
	futures[LOADDOWNERADDRESS] = promises[LOADDOWNERADDRESS].get_future();
	elevator_->setOut(-LOADAPPDOWNOUT);
	return true;
}

/* フォトインタラプタの割り込みが来る, shooterからフォトインタラプタの割り込み一定回ごとに呼び出して */
void Loader::shooterInterrupt(bool elevate, bool direction){
	if (state != State::SHOOTER_MOVE) return;
	if (elevate) {
		elevator_->setOut(SHOOTERUPPEROUT);
		promises[SHOOTERUPPERADDRESS].reset();
		futures[SHOOTERUPPERADDRESS] = promises[SHOOTERUPPERADDRESS].get_future();
		flag_counters[SHOOTERUPPERADDRESS] = ms_counter;
	} else {
		int16_t out = (direction) ? SHOOTERARMOUT : -SHOOTERARMOUT;
		arm_->setOut(out);
		promises[SHOOTERARMADDRESS].reset();
		futures[SHOOTERARMADDRESS] = promises[SHOOTERARMADDRESS].get_future();
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
			promises[SHOOTERARMADDRESS].set_success(true);
		}
	}
	if (futures[SHOOTERUPPERADDRESS].valid()){
		if (!futures[SHOOTERUPPERADDRESS].is_ready() && (ms_counter - flag_counters[SHOOTERUPPERADDRESS]) > SHOOTERUPPERSECONDS) {
			elevator_->setOut(0);
			futures[SHOOTERUPPERADDRESS].clear();
			promises[SHOOTERUPPERADDRESS].set_success(true);
		}
	}
}

/* 机から装填のアップデート, 10msごとに更新 */
void Loader::bulletUpdate(){

	if (state != State::DESK_LOAD) return;
	ms_counter += 10;
	for (size_t id = LOADDOWNERADDRESS; id < futures.size(); id++) {
		if (!futures[id].valid()) continue;
		if (!futures[id].is_ready()) continue;

		uint32_t diff_counter = ms_counter - flag_counters[id];
		size_t next = id + 1;
		if (id == LOADMOTOROUTADDRESS) {
			if (importer_upper_->isTargetReached() && importer_below_->isTargetReached()) {
				futures[id].clear();
				promises[id].set_success(true);
				shovel_->move(LOADSERVOCLOSE);
				elapsed_time = LOADSERVOSECONDS;
				promises[next].reset();
				futures[next] = promises[next].get_future();
				flag_counters[next] = ms_counter;
			}
		} else if (id == LOADMOTORINADDRESS) {
			if (importer_upper_->isTargetReached() && importer_below_->isTargetReached()) {
				futures[id].clear();
				promises[id].set_success(true);
				elevator_->setOut(LOADAPPDOWNOUT);
				elapsed_time = LOADAPPDOWNSECONDS;
				promises[next].reset();
				futures[next] = promises[next].get_future();
				flag_counters[next] = ms_counter;
			}
		} else {
			if (diff_counter > elapsed_time) {
				futures[id].clear();
				promises[id].set_success(true);
				flag_counters[next] = ms_counter;
				if (next == futures.size()){
					state = State::IDLE;
					elevator_->setOut(0);
					return;
				}
				switch (next) {
				case LOADSERVOUPPADDRESS:
					elevator_->setOut(0);
					shovel_->move(LOADSERVOOPEN);
					elapsed_time = LOADSERVOSECONDS;
					break;
				case LOADMOTOROUTADDRESS:
					importer_upper_->setTarget(LOADMOTORPOSITION);
					importer_below_->setTarget(LOADMOTORPOSITION);
					break;
				case LOADMOTORINADDRESS:
					importer_upper_->setTarget(0);
					importer_below_->setTarget(0);
					break;
				}
				promises[next].reset();
				futures[next] = promises[next].get_future();
			}
		}
		break;
	}
}

void Loader::lock(){
	locked = true;
}

void Loader::unlock(){
	locked = false;
}
