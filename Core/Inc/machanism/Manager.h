/*
 * Manager.h
 *
 *  Created on: Jul 4, 2026
 *      Author: nika-
 */

#ifndef INC_MACHANISM_MANAGER_H_
#define INC_MACHANISM_MANAGER_H_

#include <vector>

#include "main.h"
#include "Loader.h"
#include "Odometry.h"
#include "Shooter.h"
#include "Steering.h"

class Manager {
public:
	Manager(Loader* loader, Odometry* odometry, Shooter* shooter, Steering* steering, UART_HandleTypeDef* huart_);
	virtual ~Manager();

	void updatefromUART(uint8_t data);
	void updateOdometry();
	void sendSetzero(bool success);

	void sendSteeringAngle();

	bool lock();
	bool unlock();

private:
	Loader* loader_;
	Odometry* odometry_;
	Shooter* shooter_;
	Steering* steering_;

	UART_HandleTypeDef* huart_;

	std::vector<uint8_t> logger_;

	enum State {
		HEADER,
		READ,
		FOOTER
	} state  = State::HEADER;
	enum Target {
		SHOOTER,
		LOADER,
		STEER,
		LOCK,
		START,
		RESET
	} target;

	bool locked = false;
	bool resetflag = false;

	void _readSteer(uint8_t data);
	void _readLoader(uint8_t data);
	void _readShooter(uint8_t data);
	void _readLock(uint8_t data);
	void _responceShooter(bool success);
	void _responceLoader(bool success);
	void _responceLock(bool success);
};

#endif /* INC_MACHANISM_MANAGER_H_ */
