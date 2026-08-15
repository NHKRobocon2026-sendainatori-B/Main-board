/*
 * Ch4child.cpp
 *
 *  Created on: Jun 19, 2026
 *      Author: nika-
 */

#include "component/MD4ch_child.h"

/*　作ってもらったMDの子 目標値の設定はこっちで　*/
MD4ch_child::MD4ch_child()
{
	// TODO Auto-generated constructor stub

}

MD4ch_child::~MD4ch_child() {
	// TODO Auto-generated destructor stub
}

/* モードを変更 */
void MD4ch_child::setMode(Mode _mode){
	if (locked) return;
	mode = _mode;
}

/* 出力を変更 */
void MD4ch_child::setOut(int16_t _out){
	if (locked) return;
	out = _out;
}

/* モードを取得 */
Mode MD4ch_child::getMode(){
	return mode;
}

/* 出力を取得 */
int16_t MD4ch_child::getOut(){
	return out;
}

/* ロック */
void MD4ch_child::lock(){
	mode = Mode::STOP;
	out = 0;
	locked = true;
}

/* アンロック */
void MD4ch_child::unlock(){
	locked = false;
}
