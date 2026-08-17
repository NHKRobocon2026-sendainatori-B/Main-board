/*
 * Loader.cpp
 *
 *  Created on: Jul 4, 2026
 *      Author: nika-
 */

#include "Loader.h"

Loader::Loader(Servo* shovel_, MD4ch_child* arm_, MD4ch_child* elevator_, m2006* importer_upper_, m2006* importer_below_)
: shovel_(shovel_), arm_(arm_), elevator_(elevator_), importer_upper_(importer_upper_), importer_below_(importer_below_)
{
	// TODO Auto-generated constructor stub

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

/* フォトインタラプタの割り込みが来る */
void Loader::shooterInterrupt(){

}

/*机からの装填*/
bool Loader::loadbullet(){
	if (state == State::SHOOTER_MOVE) return false;
	state = State::DESK_LOAD;
	flag_counter = ms_counter;
}

/*1msごとに動かす*/
void Loader::update1ms(){
	ms_counter++;
		switch (state){
		case State::SHOOTER_MOVE:
			shooterUpdate();
			break;
		case State::DESK_LOAD:
			bulletUpdate();
			break;
		case State::IBLE:
			break;
		}
}

void Loader::lock(){
	locked = true;
}

void Loader::unlock(){
	locked = false;
}

/* private */

/*1msごとに呼び出される*/
void Loader::shooterUpdate(){

}

/*1msごとに呼び出される*/
void Loader::bulletUpdate(){

}
