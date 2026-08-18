/*
 * Loader.h
 *
 *  Created on: Jul 4, 2026
 *      Author: nika-
 */

#ifndef INC_MACHANISM_LOADER_H_
#define INC_MACHANISM_LOADER_H_

#include <map>
#include <array>
#include "main.h"
#include "Md4ch_child.h"
#include "servo.h"
#include "SpeedPIDController.h"
#include "EmbeddedPromise.hpp"

struct DeskPromise {
	int16_t time;
	Promise<bool> promise;
};

class Loader {
public:
	Loader(Servo* shovel_, MD4ch_child* arm_, MD4ch_child* elevator_, SpeedPIDController* importer_upper_, SpeedPIDController* importer_below_);
	virtual ~Loader();

	bool shooterMove(bool move);
	void shooterInterrupt(bool elevate, bool direction);
	void shooterUpdate();
	bool loadbullet();
	void bulletUpdate();

	void lock();
	void unlock();

private:
	Servo* shovel_;
	MD4ch_child* arm_;
	MD4ch_child* elevator_;
	SpeedPIDController* importer_upper_;
	SpeedPIDController* importer_below_;

	uint32_t ms_counter = 0;
	std::array<uint32_t, 8> flag_counters;

	std::array<Future<bool>, 8> futures;
	std::map<uint8_t, DeskPromise> promises;

	enum State {
		IDLE,
		SHOOTER_MOVE,
		DESK_LOAD
	} state = State::IDLE;

	bool locked = false;
};

#endif /* INC_MACHANISM_LOADER_H_ */
