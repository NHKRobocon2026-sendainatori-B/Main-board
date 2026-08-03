/*
 * Loader.cpp
 *
 *  Created on: Jul 4, 2026
 *      Author: nika-
 */

#include "Loader.h"

Loader::Loader() {
	// TODO Auto-generated constructor stub

}

Loader::~Loader() {
	// TODO Auto-generated destructor stub
}

void Loader::lock(){
	locked = true;
}

void Loader::unlock(){
	locked = false;
}
