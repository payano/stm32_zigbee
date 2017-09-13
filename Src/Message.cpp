/*
 * Message.cpp
 *
 *  Created on: Sep 13, 2017
 *      Author: johan
 */

#include "Message.h"

Message::Message():
		address(-1),
		value(-1){
	// TODO Auto-generated constructor stub
}
Message::Message(int address, int value):
		address(address),
		value(value){}

Message::~Message() {
	// TODO Auto-generated destructor stub
}

