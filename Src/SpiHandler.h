/*
 * SpiHandler.h
 *
 *  Created on: Jul 6, 2017
 *      Author: johan
 */

#ifndef SPIHANDLER_H_
#define SPIHANDLER_H_
#include "main.h"
#include "stm32f3xx_hal.h"
#include "cmsis_os.h"
#include <memory>
struct pinIO{
	GPIO_TypeDef* GPIO;
	uint16_t GPIO_Pin;
};
typedef struct pinIO pinIO;

class SpiHandler {
private:
	bool mCallback;
	SPI_HandleTypeDef *mSpi_handler;
	//pinIO &mReset, &mCs, &mPin_interrupt;
	std::unique_ptr<pinIO> pinReset, pinCs, pinInterrupt;
	uint8_t const bufferSize = 1;
	const uint8_t delay = 50;

	void _initMRF();
	void sendData(uint8_t address, uint8_t data);
	void recvData(uint8_t address, uint8_t* result);
	void sendLongData(uint16_t address, uint8_t value);
	void readLongData(uint16_t address, uint8_t* result);

	void reset(void);

public:
	SpiHandler(SPI_HandleTypeDef* spi_handler, pinIO* pinReset, pinIO* pinCs, pinIO* pinInterrupt);
	virtual ~SpiHandler();
	virtual void run();
	virtual void int_callback();
};

#endif /* SPIHANDLER_H_ */
