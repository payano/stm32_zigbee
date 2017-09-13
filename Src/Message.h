/*
 * Message.h
 *
 *  Created on: Sep 13, 2017
 *      Author: johan
 */

#ifndef MESSAGE_H_
#define MESSAGE_H_

class Message {
public:
	int address;
	int value;
	Message(int address, int value);
	Message();
	virtual ~Message();
};

#endif /* MESSAGE_H_ */
