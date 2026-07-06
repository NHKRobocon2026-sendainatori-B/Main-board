/*
 * MD4ch.h
 *
 * 開発してもらった4chMDの送信用クラスです
 *
 *  Created on: Jun 19, 2026
 *      Author: nika-
 */

#ifndef INC_COMPONENT_MD4CH_H_
#define INC_COMPONENT_MD4CH_H_

#include <vector>

#include "main.h"
#include "component/MD4ch_child.h"

class MD_4ch {
public:
	MD_4ch(CAN_HandleTypeDef* _hcan, std::vector<MD4ch_child*>* _children, uint32_t _id);
	virtual ~MD_4ch();

	void send();

private:
	CAN_HandleTypeDef* hcan;
	std::vector<MD4ch_child*>* children;
	uint32_t id;
};

#endif /* INC_COMPONENT_MD4CH_H_ */
