/*
 * MD4ch.cpp
 *
 *  Created on: Jun 19, 2026
 *      Author: nika-
 */

#include "component/MD4ch.h"

/* 開発してもらったMDのmanager */
MD_4ch::MD_4ch(CAN_HandleTypeDef* _hcan, std::vector<MD4ch_child*>* _children, uint32_t _id)
: hcan(_hcan), children(_children), id(_id)
{
	// TODO Auto-generated constructor stub
}

MD_4ch::~MD_4ch() {
	// TODO Auto-generated destructor stub
}

/*　データをCANで送る　一定間隔で呼び出す　*/
void MD_4ch::send(){
	uint8_t packed_data[8] = {0};
	for (size_t i = 0; i < 4 && i < children->size(); i++){
		//モードの確認
		int8_t mode = 0;
		switch((*children)[i]->getMode()){
			case Mode::STOP:
				mode = 0;
				break;
			case Mode::POSITION:
				mode = 1;
				break;
			case Mode::OPENLOOP:
				mode = 2;
				break;
			case Mode::ENCODER:
				mode = 3;
				break;
		}

		//出力の取得
		int16_t out = (*children)[i]->getOut();

		//データの作製
		packed_data[i * 2] = (mode << 6 & 0xc0) | (out >> 8 & 0x3f);
		packed_data[i * 2 + 1] = out & 0xFF;
	}

	//送信フェーズ
	CAN_TxHeaderTypeDef txHeader;
	uint32_t txMailbox;
	txHeader.IDE = CAN_ID_STD;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.DLC = 8;
	txHeader.StdId = id;
	HAL_CAN_AddTxMessage(hcan, &txHeader, packed_data, &txMailbox);
}

