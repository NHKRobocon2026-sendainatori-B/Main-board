/*
 * Loader.h
 *
 *  Created on: Jul 4, 2026
 *      Author: nika-
 */

#ifndef INC_MACHANISM_LOADER_H_
#define INC_MACHANISM_LOADER_H_

#include "main.h"

class Loader {
public:
	Loader();
	virtual ~Loader();

	void lock();
	void unlock();

private:
	bool locked = false;
};

#endif /* INC_MACHANISM_LOADER_H_ */
