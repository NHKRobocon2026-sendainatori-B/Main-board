/*
 * Manager.h
 *
 *  Created on: Jul 4, 2026
 *      Author: nika-
 */

#ifndef INC_MACHANISM_MANAGER_H_
#define INC_MACHANISM_MANAGER_H_

#include "main.h"
#include "Loader.h"
#include "Odometry.h"
#include "Shooter.h"
#include "Steering.h"

class Manager {
public:
	Manager(Loader* loader, Odometry* odometry, Shooter* shooter, Steering* steering);
	virtual ~Manager();

	void updatefromUART(uint8_t data);
	void updateOdometry();

	void lock();
	void unlock();

private:
	Loader* loader_;
	Odometry* odometry_;
	Shooter* shooter_;
	Steering* steering_;

	UART_HandleTypeDef* huart_;

	bool locked = false;
};

#endif /* INC_MACHANISM_MANAGER_H_ */
