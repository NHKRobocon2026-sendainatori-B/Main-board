/*
 * Encoder.h
 *
 *  Created on: Jul 4, 2026
 *      Author: nika-
 */

#ifndef INC_COMPONENT_PRIVATE_ENCODER_H_
#define INC_COMPONENT_PRIVATE_ENCODER_H_

#include "main.h"

class Encoder {
public:
	Encoder();
	virtual ~Encoder();

	/* 今の角度を取得 */
	virtual int32_t getAngle() = 0;
	/* 今の位置を0にする */
	virtual void setZero() = 0;
};

#endif /* INC_COMPONENT_PRIVATE_ENCODER_H_ */
