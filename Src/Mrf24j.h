/*
 * Mrf24j.h
 *
 *  Created on: Jul 6, 2017
 *      Author: johan
 */

#ifndef MRF24J_H_
#define MRF24J_H_
#include "main.h"
#include "stm32f3xx_hal.h"
#include "cmsis_os.h"
#include <memory>
struct pinIO{
	GPIO_TypeDef* GPIO;
	uint16_t GPIO_Pin;
};
typedef struct pinIO pinIO;

class Mrf24j {
private:
	bool mCallback = false;
	SPI_HandleTypeDef *mSpi_handler;
	//pinIO &mReset, &mCs, &mPin_interrupt;
	const std::unique_ptr<pinIO> pinReset, pinCs, pinInterrupt;
	uint8_t const bufferSize = 1;
	const uint8_t delay = 50;

	void _initMRF();
	void sendData(uint8_t address, uint8_t value);
	void recvData(uint8_t address, uint8_t* result);
	void sendLongData(uint16_t address, uint8_t value);
	void readLongData(uint16_t address, uint8_t* result);

	void reset(void);

public:
	Mrf24j(SPI_HandleTypeDef* spi_handler, std::unique_ptr<pinIO> pinReset, std::unique_ptr<pinIO> pinCs, std::unique_ptr<pinIO> pinInterrupt);
	Mrf24j(SPI_HandleTypeDef* spi_handler, std::unique_ptr<pinIO> pinReset);
	virtual ~Mrf24j();
	virtual void run();
	virtual void int_callback();
};

#endif /* MRF24J_H_ */
