/*
 * m2006manager.h
 *
 *  Created on: Jul 2, 2026
 *      Author: nika-
 */

#ifndef INC_COMPONENT_M2006MANAGER_H_
#define INC_COMPONENT_M2006MANAGER_H_

#include <vector>
#include <main.h>
#include "component/m2006.h"

class m2006_manager {
public:
	m2006_manager(std::vector<m2006*>* _children, CAN_HandleTypeDef* _hcan);
	virtual ~m2006_manager();

	void sendtoCAN();
	void update(); //入っているm2006を全てPIDを計算
	void updatefromCAN(uint8_t data[8], uint32_t id);
private:
	CAN_HandleTypeDef* hcan;
	std::vector<m2006*>* children;
};

#endif /* INC_COMPONENT_M2006MANAGER_H_ */
