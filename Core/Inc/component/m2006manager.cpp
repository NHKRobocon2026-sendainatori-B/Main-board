/*
 * m2006manager.cpp
 *
 *  Created on: Jul 2, 2026
 *      Author: nika-
 */

#include "component/m2006manager.h"

/* m2006のメッセージを送るマネージャー */
m2006_manager::m2006_manager(std::vector<m2006*>* _children, CAN_HandleTypeDef* _hcan)
: hcan(_hcan), children(_children)
{
	// TODO Auto-generated constructor stub

}

m2006_manager::~m2006_manager() {
	// TODO Auto-generated destructor stub
}

/* CANを送る 一定間隔で呼び出す */
void m2006_manager::sendtoCAN() {
	uint8_t txData200[8] = {0}; // モーター1~4
	uint8_t txData1FF[8] = {0}; // モーター5~8

	bool hasGroup1 = false;
	bool hasGroup2 = false;

	for (size_t i = 0; i < 8 && i < children->size(); i++){
		int16_t cmd = (*children)[i]->getTargetCurrent();
		if (i >= 0 && i < 4) {
			// 1~4番目：0x200用のバッファに詰める
			int idx = i;
		   	txData200[idx * 2]     = (cmd >> 8) & 0xFF;
		    txData200[idx * 2 + 1] = cmd & 0xFF;
		    hasGroup1 = true;
		} else if(i >= 4 && i < 8) {
		    // 5~8番目：0x1FF用のバッファに詰める (i=4が5番目に対応)
		    int idx = i - 4;
		    txData1FF[idx * 2]     = (cmd >> 8) & 0xFF;
		    txData1FF[idx * 2 + 1] = cmd & 0xFF;
		    hasGroup2 = true;
		}
	}

	uint32_t txMailbox;
	CAN_TxHeaderTypeDef txHeader;
	txHeader.IDE = CAN_ID_STD;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.DLC = 8;

	// グループ1 (1~4番) が存在すれば送信
	if (hasGroup1) {
	    txHeader.StdId = 0x200;
	    HAL_CAN_AddTxMessage(hcan, &txHeader, txData200, &txMailbox);
	}

	// グループ2 (5~8番) が存在すれば送信
	if (hasGroup2) {
	    txHeader.StdId = 0x1FF;
	    HAL_CAN_AddTxMessage(hcan, &txHeader, txData1FF, &txMailbox);
	}
}

/* CANからの情報を格納 dataとidをそのまま入れて */
void m2006_manager::updatefromCAN(uint8_t data[8], uint32_t id){
	int32_t idx = id - 0x201;
	if (idx < 0 || idx >= static_cast<int32_t>(children->size())) {
		return;
	}
	(*children)[idx]->updateFromCAN(data);
}
