/*
 * SpiHandler.cpp
 *
 *  Created on: Jul 6, 2017
 *      Author: johan
 */

#include "SpiHandler.h"

SpiHandler::SpiHandler(SPI_HandleTypeDef* spi_handler, pinIO& pin_reset, pinIO& pin_chip_select, pinIO& pin_interrupt):
	mSpi_handler(spi_handler), mReset(pin_reset), mCs(pin_chip_select), mPin_interrupt(pin_interrupt), mCallback(false)
{
	HAL_SPI_Init(mSpi_handler);
}


SpiHandler::~SpiHandler() {
	// TODO Auto-generated destructor stub
}

void SpiHandler::int_callback(){
	mCallback = true;
}

void SpiHandler::reset(void){
	//https://github.com/karlp/Mrf24j40-arduino-library/blob/master/mrf24j.cpp#L1

}


void SpiHandler::run(){

}


void SpiHandler::sendData(uint8_t address, uint8_t data){
	//Mask data correctly before sending.
	address = address << 1;
	address &= 0b01111111;
	address |= 0x01;

	//Enable CS Pin, make MRF aware of incoming message.
	HAL_GPIO_WritePin(MRF_CS_GPIO_Port, MRF_CS_Pin, GPIO_PIN_RESET);

	//Send address and data
	HAL_SPI_Transmit(mSpi_handler,&address, 1, 1);
	HAL_SPI_Transmit(mSpi_handler,&data, 1, 1);

	//Disable transfer to mrf.
	HAL_GPIO_WritePin(MRF_CS_GPIO_Port, MRF_CS_Pin, GPIO_PIN_SET);

	//Delay 1 ms
	HAL_Delay(1);

}
void  SpiHandler::recvData(uint8_t address, uint8_t* result){
	//Mask data correctly before sending.
	address = address << 1;
	address &= 0b01111110;

	//Enable CS Pin, make MRF aware of incoming message.
	HAL_GPIO_WritePin(MRF_CS_GPIO_Port, MRF_CS_Pin, GPIO_PIN_RESET);

	//Send address and receive data
	HAL_SPI_Transmit(mSpi_handler,&address, bufferSize, 1);
	HAL_SPI_Receive(mSpi_handler,result,bufferSize, 1);

	//Disable transfer to mrf.
	HAL_GPIO_WritePin(MRF_CS_GPIO_Port, MRF_CS_Pin, GPIO_PIN_SET);

	//Delay 1 ms
	HAL_Delay(1);
}

void SpiHandler::sendLongData(uint16_t address, uint8_t value)
{
	//Mask data correctly before sending.
	uint8_t data1 = (((uint8_t)(address>>3))&0x7F)|0x80;
	uint8_t data2 = (((uint8_t)(address<<5))&0xE0)|0x10;

	//Enable CS Pin, make MRF aware of incoming message.
	HAL_GPIO_WritePin(MRF_CS_GPIO_Port, MRF_CS_Pin, GPIO_PIN_RESET);

	//Send address and data
	HAL_SPI_Transmit(mSpi_handler, &data1, 1, 1);
	HAL_SPI_Transmit(mSpi_handler, &data2, 1, 1);
	HAL_SPI_Transmit(mSpi_handler, &value, 1, 1);

	//Disable transfer to mrf.
	HAL_GPIO_WritePin(MRF_CS_GPIO_Port, MRF_CS_Pin, GPIO_PIN_SET);

	//Delay 1 ms
	HAL_Delay(1);
}
void SpiHandler::readLongData(uint16_t address, uint8_t* result){
		//Mask data correctly before reading.
		uint8_t address1 = ((address>>3)&0x7F)|0x80;
		uint8_t address2 = ((address<<5)&0xE0);

		//Enable CS Pin, make MRF aware of incoming message.
		HAL_GPIO_WritePin(MRF_CS_GPIO_Port, MRF_CS_Pin, GPIO_PIN_RESET);

		//Send address and receieve data
		HAL_SPI_Transmit(mSpi_handler,&address1, 1, 1);
		HAL_SPI_Transmit(mSpi_handler, &address2, 1, 1);
		HAL_SPI_Receive(mSpi_handler, result, 1, 1);

		//Disable transfer to mrf.
		HAL_GPIO_WritePin(MRF_CS_GPIO_Port, MRF_CS_Pin, GPIO_PIN_SET);

		//Delay 1 ms
		HAL_Delay(1);
}



void SpiHandler::_initMRF(){

}
