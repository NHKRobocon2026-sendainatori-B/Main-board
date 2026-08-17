/*
 * Loader.h
 *
 *  Created on: Jul 4, 2026
 *      Author: nika-
 */

#ifndef INC_MACHANISM_LOADER_H_
#define INC_MACHANISM_LOADER_H_

#include "main.h"
#include "Md4ch_child.h"
#include "servo.h"
#include "m2006.h"

class Loader {
public:
	Loader(Servo* shovel_, MD4ch_child* arm_, MD4ch_child* elevator_, m2006* importer_upper_, m2006* importer_below_);
	virtual ~Loader();

	bool shooterMove(bool move);
	void shooterInterrupt();
	bool loadbullet();

	void update1ms();

	void lock();
	void unlock();

private:
	void shooterUpdate();
	void bulletUpdate();

	Servo* shovel_;
	MD4ch_child* arm_;
	MD4ch_child* elevator_;
	m2006* importer_upper_;
	m2006* importer_below_;

	int32_t ms_counter = 0;
	int32_t flag_counter = 0;

	enum State {
		IBLE,
		SHOOTER_MOVE,
		DESK_LOAD
	} state = State::IBLE;

	enum ShooterState {

	} shooter;

	enum BulletState {

	} bullet;

	bool locked = false;
};

#endif /* INC_MACHANISM_LOADER_H_ */
