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

	void lock();
	void unlock();

private:
	Servo* shovel_;
	MD4ch_child* arm_;
	MD4ch_child* elevator_;
	m2006* importer_upper_;
	m2006* importer_below_;

	enum State {
		IBLE,
		SHOOTER_LOAD,
		DESK_LOAD
	} state;

	bool locked = false;
};

#endif /* INC_MACHANISM_LOADER_H_ */
