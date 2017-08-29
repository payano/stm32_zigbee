/*
 * SpiHandler.cpp
 *
 *  Created on: Jul 6, 2017
 *      Author: johan
 */

#include "SpiHandler.h"
//#include "MRF24J40.h"
// MRF20J40 Short address control register mapping.
#define RXMCR     0x00
#define PANIDL    0x01
#define PANIDH    0x02
#define SADRL     0x03
#define SADRH     0x04
#define EADR0     0x05
#define EADR1     0x06
#define EADR2     0x07
#define EADR3     0x08
#define EADR4     0x09
#define EADR5     0x0a
#define EADR6     0x0b
#define EADR7     0x0c
#define RXFLUSH   0x0d

#define TXNMTRIG  0x1b
#define TXSR      0x24

#define ISRSTS    0x31
#define INTMSK    0x32
#define TRISGPIO  0x34

#define RFCTL     0x36

#define BBREG2    0x3A

#define BBREG6    0x3E
#define RSSITHCCA 0x3F

// MRF20J40 Long address control register mapping.
#define RFCTRL0   0x200

#define RFCTRL2   0x202
#define RFCTRL3   0x203

#define RFCTRL6   0x206
#define RFCTRL7   0x207
#define RFCTRL8   0x208

#define CLKINTCR  0x211
#define CLCCTRL   0x220


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

	osDelay(10);
	//HAL_GPIO_WritePin(MRF_RESET_GPIO_Port, MRF_RESET_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(mReset.GPIO, mReset.GPIO_Pin, GPIO_PIN_RESET);
	osDelay(10);
    //HAL_GPIO_WritePin(MRF_RESET_GPIO_Port, MRF_RESET_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(mReset.GPIO, mReset.GPIO_Pin, GPIO_PIN_SET);
	osDelay(20);


    //osDelay(200);  // from manual
}


void SpiHandler::run(){

	//_initMRF();
	//uint8_t aTxBuffer = 0x0F;
	//uint8_t aRxBuffer = 0;
	//uint8_t const bufferSize = 1;
	HAL_GPIO_WritePin(mCs.GPIO, mCs.GPIO_Pin, GPIO_PIN_SET);
	reset();
	//recvData(READ_SOFTRST);_initMRF();
	sendData(RXFLUSH, 0x01);
	HAL_Delay(5);

	for(;;){

		//TEST THIS:
		//WITH INTERRUPT
		//https://github.com/karlp/Mrf24j40-arduino-library/blob/master/mrf24j.cpp
		//MRF chipset: http://ww1.microchip.com/downloads/en/DeviceDoc/39776C.pdf

		//Source for init: https://developer.mbed.org/users/hilgo/code/MRF24J40/file/55d2672c4708/MRF24J40.cpp

		sendData(MRF_PACON2, 0x98); // – Initialize FIFOEN = 1 and TXONTS = 0x6.
		sendData(MRF_TXSTBL, 0x95); // – Initialize RFSTBL = 0x9.

		sendLongData(MRF_RFCON0, 0x03); // – Initialize RFOPT = 0x03.
		sendLongData(MRF_RFCON1, 0x01); // – Initialize VCOOPT = 0x02.
		sendLongData(MRF_RFCON2, 0x80); // – Enable PLL (PLLEN = 1).
		sendLongData(MRF_RFCON6, 0x90); // – Initialize TXFIL = 1 and 20MRECVR = 1.
		sendLongData(MRF_RFCON7, 0x80); // – Initialize SLPCLKSEL = 0x2 (100 kHz Internal oscillator).
		sendLongData(MRF_RFCON8, 0x10); // – Initialize RFVCO = 1.
		sendLongData(MRF_SLPCON1, 0x21); // – Initialize CLKOUTEN = 1 and SLPCLKDIV = 0x01.

		//  Configuration for nonbeacon-enabled devices (see Section 3.8 “Beacon-Enabled and
		//  Nonbeacon-Enabled Networks”):
		sendData(MRF_BBREG2, 0x80); // Set CCA mode to ED
		sendData(MRF_CCAEDTH, 0x60); // – Set CCA ED threshold.
		sendData(MRF_BBREG6, 0x40); // – Set appended RSSI value to RXFIFO.


		//set_interrupts();
		sendData(MRF_INTCON, 0b11110110);
		uint8_t channel = 12;
		//set_channel(12);
		sendData(MRF_RFCON0, (((channel - 11) << 4) | 0x03));


		// max power is by default.. just leave it...
		// Set transmitter power - See “REGISTER 2-62: RF CONTROL 3 REGISTER (ADDRESS: 0x203)”.
		sendData(MRF_RFCTL, 0x04); //  – Reset RF state machine.
		sendData(MRF_RFCTL, 0x00); // part 2


		recvData(MRF_INTCON);
		osDelay(1); // delay at least 192usec

		// Reset RF module.
		sendData(RFCTL, 0x04);
		sendData(RFCTL, 0x00);
		sendData(RFCTL, 0x00);

		sendData(PANIDL, 0xAA);
		sendData(PANIDH, 0xAA);
		sendData(SADRL, 0xAA);
		sendData(SADRH, 0xAA);

		// Flush RX fifo.

		recvData(RXFLUSH);


		// Write MAC addresses here. We don't care.

		sendLongData(RFCTRL2, 0x80);  // Enable RF PLL.

		sendLongData(RFCTRL3, 0x00);  // Full power.
		sendLongData(RFCTRL6, 0x80);  // Enable TX filter (recommended)
		sendLongData(RFCTRL8, 0x10);  // Enhanced VCO (recommended)

		sendData(BBREG2,0x78);   // Clear Channel Assesment use carrier sense.
		sendData(BBREG6,0x40);   // Calculate RSSI for Rx packet.
		sendData(RSSITHCCA,0x00);// RSSI threshold for CCA.

		sendLongData(RFCTRL0, 0x00);  // Channel 11.

		readLongData(RFCTRL0);

		sendData(RXMCR, 0x01); // Don't check address upon reception.
		//MrfWriteShort(RXMCR, 0x00); // Check address upon reception.
		//
		//// Reset RF module with new settings.
		sendData(RFCTL, 0x04);
		sendData(RFCTL, 0x00);
		// END OF MBED CODE


		//
		//uint8_t data[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a };
		//uint8_t i;
		//uint8_t length = 11;
		//
		//sendLongData(0x000, 0);   // No addresses in header.
		//sendLongData(0x001, length); // 11 bytes
		//for(i=0; i<length; i++)
		//	sendLongData(0x002+i, data[i]);
		//
		//recvData(TXNMTRIG);
		//sendData(TXNMTRIG, 0x01);
		//recvData(TXNMTRIG);
		//recvData(TXNMTRIG);
		//recvData(TXNMTRIG);




		//			  osDelay(20);
		//HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(MRF_CS_GPIO_Port, MRF_CS_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_SET);
		//HAL_SPI_TransmitReceive(mSpi_handler, &aTxBuffer, &aRxBuffer, bufferSize, 50);
		//if(!mCallback){
		//reset();
		//sendData(0x2a, 0x07); //SOFTRST
		//sendData(0x18, 0x98); //TXSTBL
		//sendData(0x2e, 0x95); //Initialize RFSTBL = 0x9.
		//sendLongData(0x200, 0x03);
		//sendLongData(0x201, 0x01);
		//sendLongData(0x202, 0x80);
		//sendLongData(0x206, 0x90);
		//sendLongData(0x207, 0x80);
		//sendLongData(0x208, 0x10);
		//sendLongData(0x220, 0x21);
		//
		//
		//sendData(0x41, 0x03); //RFCON0
		//sendLongData(0x301, 0x01);
		//sendData(); //RFCON1

		/*
			1. SOFTRST (0x2A) = 0x07 – Perform a software Reset. The bits will be automatically cleared to ‘0’ by hardware.
			2. PACON2 (0x18) = 0x98 – Initialize FIFOEN = 1 and TXONTS = 0x6.
			3. TXSTBL (0x2E) = 0x95 – Initialize RFSTBL = 0x9.
			4. RFCON0 (0x200) = 0x03 – Initialize RFOPT = 0x03.
			5. RFCON1 (0x201) = 0x01 – Initialize VCOOPT = 0x02.
			6. RFCON2 (0x202) = 0x80 – Enable PLL (PLLEN = 1).
			7. RFCON6 (0x206) = 0x90 – Initialize TXFIL = 1 and 20MRECVR = 1.
			8. RFCON7 (0x207) = 0x80 – Initialize SLPCLKSEL = 0x2 (100 kHz Internal oscillator).
			9. RFCON8 (0x208) = 0x10 – Initialize RFVCO = 1.
			10. SLPCON1 (0x220) = 0x21 – Initialize CLKOUTEN = 1 and SLPCLKDIV = 0x01
		 */

		//sendData(0x3a, 0x80);
		//sendData(0x3F, 0x60);
		//sendData(0x3e, 0x40);
		//sendData(0x36, 0x04);
		//recvData(0x36);
		//sendData(0x36,0x00);
		//recvData(0x36);
		//recvData(0x3a);
		osDelay(20);


		/*
			Configuration for nonbeacon-enabled devices (see Section 3.8 “Beacon-Enabled and Nonbeacon-Enabled
			Networks”):
			11. BBREG2 (0x3A) = 0x80 – Set CCA mode to ED.
			12. CCAEDTH = 0x60 – Set CCA ED threshold.
			13. BBREG6 (0x3E) = 0x40 – Set appended RSSI value to RXFIFO.
			14. Enable interrupts – See Section 3.3 “Interrupts”.
			15. Set channel – See Section 3.4 “Channel Selection”.
			16. Set transmitter power - See “REGISTER 2-62: RF CONTROL 3 REGISTER (ADDRESS: 0x203)”.
			17. RFCTL (0x36) = 0x04 – Reset RF state machine.
			18. RFCTL (0x36) = 0x00.
			19. Delay at least 192 μs.

16. Set transmitter power - See “REGISTER 2-62: RF CONTROL 3 REGISTER (ADDRESS: 0x203)”.
17. RFCTL (0x36) = 0x04 – Reset RF state machine.
18. RFCTL (0x36) = 0x00.
19. Delay at least 192 μs.
		 */


		//HAL_SPI_TransmitReceive(mSpi_handler, &aTxBuffer, &aRxBuffer, bufferSize, 50);


		//osDelay(20);


		//			  aTxBuffer = ~aTxBuffer;
		//			  //HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
		//			  HAL_GPIO_WritePin(MRF_CS_GPIO_Port, MRF_CS_Pin, GPIO_PIN_RESET);
		//			  HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET);
		//			  HAL_SPI_TransmitReceive(mSpi_handler, &aTxBuffer, &aRxBuffer, bufferSize, 50);
	}


	/* USER CODE BEGIN 2 */
	//  uint8_t data = 0xA5;
	//HAL_SPI_Init(mSpi_handler);
	//uint8_t return_transmit;
	//	  uint8_t aTxBuffer = 0xF0;
	//	  uint8_t aRxBuffer = 0x20;
	//	  int const BUFFERSIZE = 1;

	//uint32_t i = 0;
	//uint8_t SPI1_Tx_Data  = 0xD000;
	//uint8_t  SPI1_Rx_Data  = 0;
	//	for(;;)
	//	{
	//		  //HAL_Delay(50);
	//		osDelay(100);
	//		  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	//		  HAL_SPI_TransmitReceive(mSpi_handler, &aTxBuffer, &aRxBuffer, BUFFERSIZE, 50);
	//		  HAL_SPI_TransmitReceive(mSpi_handler, &aTxBuffer, &aRxBuffer, BUFFERSIZE, 50);
	//			osDelay(100);
	//			  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	//		  		  //HAL_Delay(200);
	//HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	//  HAL_SPI_Transmit(mSpi_handler, &data, BUFFERSIZE, 15);
	//HAL_SPI_Transmit()
	//data = ~data;
	//osDelay(20);
	//HAL_Delay(200);
	//HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	//HAL_SPI_Transmit(&hspi3, &data, BUFFERSIZE, 15);
	//return_transmit = HAL_SPI_Transmit(&hspi2, &data, 1, 500);
	//HAL_Delay(50);
	//HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);








	//HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	//osDelay(1);
	//}
}


uint8_t SpiHandler::recvData(uint8_t address){

	uint8_t result = 0;
	address = address << 1;
	address &= 0b01111110;

	uint8_t new_address[] = {address, 0x0};

	//HAL_Delay(1);
	HAL_GPIO_WritePin(MRF_CS_GPIO_Port, MRF_CS_Pin, GPIO_PIN_RESET);
	//HAL_Delay(1);
	//HAL_SPI_TransmitReceive(mSpi_handler,new_address, result, 2, 0);

	HAL_SPI_Transmit(mSpi_handler,&address, bufferSize, 1);
	HAL_SPI_Receive(mSpi_handler,&result,bufferSize, 1);
	//HAL_Delay(1);
	HAL_GPIO_WritePin(MRF_CS_GPIO_Port, MRF_CS_Pin, GPIO_PIN_SET);
	//HAL_Delay(1);

	//osDelay(1);
	//osDelay(5);
	return 0; //TOdO FIX THIS
}

// writes "value" to radio at long "address"
void SpiHandler::sendLongData(uint16_t address, uint8_t value)
{

	//HAL_SPI_TransmitReceive(mSpi_handler, &data, &result, sizeof(data), delay);
	uint8_t data1 = (((uint8_t)(address>>3))&0x7F)|0x80;
	uint8_t data2 = (((uint8_t)(address<<5))&0xE0)|0x10;

	uint8_t datan[] = {data1, data2,value };

	//HAL_Delay(1);
	HAL_GPIO_WritePin(MRF_CS_GPIO_Port, MRF_CS_Pin, GPIO_PIN_RESET);

	//HAL_SPI_Transmit(mSpi_handler,&data1, 3, 0);
	HAL_SPI_Transmit(mSpi_handler, &data1, 1, 1);
	HAL_SPI_Transmit(mSpi_handler, &data2, 1, 1);
	HAL_SPI_Transmit(mSpi_handler, &value, 1, 1);


	//spiPut((((UINT8)(address>>3))&0x7F)|0x80);
	//spiPut((((UINT8)(address<<5))&0xE0)|0x10);
	//spiPut(value);
	HAL_GPIO_WritePin(MRF_CS_GPIO_Port, MRF_CS_Pin, GPIO_PIN_SET);
	//HAL_Delay(1);

}
uint8_t SpiHandler::readLongData(uint16_t address){
		uint8_t toReturn;
		uint8_t address1 = ((address>>3)&0x7F)|0x80;
		uint8_t address2 = ((address<<5)&0xE0);

		//HAL_Delay(1);
		HAL_GPIO_WritePin(MRF_CS_GPIO_Port, MRF_CS_Pin, GPIO_PIN_RESET);

		HAL_SPI_Transmit(mSpi_handler,&address1, 1, 1);
		HAL_SPI_Transmit(mSpi_handler, &address2, 1, 1);
		HAL_SPI_Receive(mSpi_handler, &toReturn, 1, 1);

		HAL_GPIO_WritePin(MRF_CS_GPIO_Port, MRF_CS_Pin, GPIO_PIN_SET);
		//HAL_Delay(1);


		return toReturn;
}
//
//void SpiHandler::sendLongData(uint16_t address, uint8_t data){
//
//
//	uint8_t firstbyte = 0x0;
//
//	firstbyte = address >> 4;
//	firstbyte |= 0x80;
//
//	uint8_t secondbyte = address & 0x0f;
//	secondbyte |= 0x01;
//	secondbyte = secondbyte << 4;
//
//	uint8_t datan[] = {firstbyte, secondbyte, data};
//
//
//	HAL_GPIO_WritePin(MRF_CS_GPIO_Port, MRF_CS_Pin, GPIO_PIN_RESET);
//
//	//__SPI2_CLK_ENABLE();
//
//
//	HAL_SPI_Transmit(mSpi_handler,&firstbyte, sizeof(firstbyte), delay);
//	//__SPI1_CLK_ENABLE();
//	HAL_SPI_Transmit(mSpi_handler,&secondbyte, sizeof(secondbyte), delay);
//	//__SPI1_CLK_ENABLE();
//	HAL_SPI_Transmit(mSpi_handler,&data, sizeof(data), delay);
//
//	//__SPI2_CLK_DISABLE();
//
//	HAL_GPIO_WritePin(MRF_CS_GPIO_Port, MRF_CS_Pin, GPIO_PIN_SET);
//
//}
void SpiHandler::sendData(uint8_t address, uint8_t data){


	//TODO: fix sendData.

	//HAL_GPIO_WritePin(MRF_CS_GPIO_Port, MRF_CS_Pin, GPIO_PIN_SET);
	//osDelay(1);
	//HAL_GPIO_WritePin(MRF_CS_GPIO_Port, MRF_CS_Pin, GPIO_PIN_RESET);

//	osDelay(5);
//	HAL_SPI_Transmit(mSpi_handler,&address, bufferSize, delay);
//	osDelay(5);
//	HAL_SPI_Transmit(mSpi_handler,&data, bufferSize, delay);

	//uint8_t message[] = {data, address};
	//uint8_t result = 0x00;

	address = address << 1;
	address &= 0b01111111;
	address |= 0x01;

	uint8_t dataaddress[] = { address, data } ;
	//HAL_Delay(1);
	HAL_GPIO_WritePin(MRF_CS_GPIO_Port, MRF_CS_Pin, GPIO_PIN_RESET);
	//HAL_Delay(1);
	//HAL_SPI_Transmit(mSpi_handler,dataaddress, 2, 0);
	HAL_SPI_Transmit(mSpi_handler,&address, 1, 1);
	HAL_SPI_Transmit(mSpi_handler,&data, 1, 1);
	//HAL_Delay(1);
	HAL_GPIO_WritePin(MRF_CS_GPIO_Port, MRF_CS_Pin, GPIO_PIN_SET);
	//HAL_Delay(1);



//	uint8_t datan[] = {address, data};
//	HAL_GPIO_WritePin(MRF_CS_GPIO_Port, MRF_CS_Pin, GPIO_PIN_RESET);
//	osDelay(1);
//	HAL_SPI_TransmitReceive(mSpi_handler, datan, &result, sizeof(address), delay);
//
//	HAL_SPI_TransmitReceive(mSpi_handler, &data, &result, sizeof(data), delay);
//
//	HAL_GPIO_WritePin(MRF_CS_GPIO_Port, MRF_CS_Pin, GPIO_PIN_SET);
	//osDelay(1);

	//HAL_SPI_Transmit(mSpi_handler,message, 1, delay);
	//osDelay(5);
	//HAL_SPI_Transmit(mSpi_handler,message, 1, delay);
	HAL_Delay(1);

}


void SpiHandler::_initMRF(){
	const uint8_t delay = 10;
	const uint8_t bufferSize = 1;


	//osDelay(delay);
	//HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout)
	//sendData(0xf0, 0x0f);

//	sendData(WRITE_SOFTRST, 0x07);
//	recvData(READ_SOFTRST);
//	recvData(READ_SOFTRST);

	/* do a soft reset */
	//sendData(0x0f,0xf0);		// reset everything (power, baseband, MAC) (also does wakeup if in sleep)
	//recvData(READ_SOFTRST);

	//SOFT RESET:
	sendData(WRITE_SOFTRST, 0x00);

//	int i;
//	do
//	{
//		i = recvData(READ_SOFTRST);
//
//		//if (CT_TICKS_SINCE(radioReset) > MS_TO_CORE_TICKS(50))		// if no reset in a reasonable time
//			//return 0;												// then there is no radio hardware
//		osDelay(1);
//	}
//	while((i&0x07) != (uint8_t)0x00);
//
//	for(;;){
//		sendData(0xf0, 0x0f);
//	}
//
	//sendData(READ_SOFTRST, 0x00);


	//uint8_t i;
//	do
//	{
		//sendData(WRITE_SOFTRST,0x07);		// reset everything (power, baseband, MAC) (also does wakeup if in sleep)
//		i = recvData(READ_SOFTRST);

//		if (CT_TICKS_SINCE(radioReset) > MS_TO_CORE_TICKS(50))		// if no reset in a reasonable time
//			return 0;												// then there is no radio hardware
//	}
//	while((i&0x07) != (uint8_t)0x00);   	// wait for hardware to clear reset bits



//
//	do
//	{
//		i = lowRead(READ_SOFTRST);
//
//		if (CT_TICKS_SINCE(radioReset) > MS_TO_CORE_TICKS(50))		// if no reset in a reasonable time
//			return 0;												// then there is no radio hardware
//	}
//	while((i&0x07) != (UINT8)0x00);

	//osDelay(30);






	/* do a soft reset */
  //HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout)
  //lowWrite(WRITE_SOFTRST,0x07);		// reset everything (power, baseband, MAC) (also does wakeup if in sleep)
//  data = WRITE_SOFTRST;
//  HAL_SPI_Transmit(mSpi_handler, &data, 1, 50);
//
//  osDelay(delay);
//  data = 0x07;
//  HAL_SPI_Transmit(mSpi_handler, &data, 1, 50);
//  osDelay(delay);
//  int i = HAL_SPI_Receive(mSpi_handler, &data, 1, 50);
//
//  //osDelay(delay*delay);
//
//	//lowWrite(WRITE_RXFLUSH,0x01);		// flush the RX fifo, leave WAKE pin disabled
//  data = WRITE_RXFLUSH;
//  HAL_SPI_Transmit(mSpi_handler, &data, 1, 50);
//  osDelay(delay);
//  data = 0x01;
//  HAL_SPI_Transmit(mSpi_handler, &data, 1, 50);
////}
//
//  osDelay(delay);
//
//
//  data = WRITE_SADRL;
//  HAL_SPI_Transmit(mSpi_handler, &data, 1, 50);
  /*
	lowWrite(WRITE_SADRL,BYTEPTR(shortAddress)[0]);
	lowWrite(WRITE_SADRH,BYTEPTR(shortAddress)[1]);

	lowWrite(WRITE_PANIDL,BYTEPTR(panID)[0]);
	lowWrite(WRITE_PANIDH,BYTEPTR(panID)[1]);

	for(i=0;i<sizeof(longAddress);i++){	// program long MAC address
		lowWrite(WRITE_EADR0+i*2,BYTEPTR(longAddress)[i]);
	}



	RadioSetAddress(RadioStatus.MyShortAddress, RadioStatus.MyLongAddress, RadioStatus.MyPANID);

	highWrite(RFCTRL0,0x03);			// RFOPT=0x03
	highWrite(RFCTRL1,0x02);			// VCOOPT=0x02, per datasheet
	highWrite(RFCTRL2,0x80);			// PLL enable
	highWrite(RFCTRL3, TX_POWER);		// set transmit power
	highWrite(RFCTRL6,0x90);			// TXFILter on, 20MRECVR set to < 3 mS
	highWrite(RFCTRL7,0x80);			// sleep clock 100 kHz internal
	highWrite(RFCTRL8,0x10);			// RFVCO to 1

	highWrite(SCLKDIV, 0x21);			// CLKOUT disabled, sleep clock divisor is 2

	lowWrite(WRITE_BBREG2,0x80);		// CCA energy threshold mode
	lowWrite(WRITE_BBREG6,0x40);		// RSSI on every packet
	lowWrite(WRITE_RSSITHCCA,0x60);		// CCA threshold ~ -69 dBm

	#if defined(ENABLE_PA_LNA)
		highWrite(TESTMODE, 0x0F);		// setup for PA_LNA mode control
	#endif

	lowWrite(WRITE_FFOEN, 0x98);		// PACON2, per datasheet init
	lowWrite(WRITE_TXPEMISP, 0x95);  	// TXSTBL; RFSTBL=9, MSIFS-5

	while((highRead(RFSTATE)&0xA0) != 0xA0);	// wait till RF state machine in RX mode

	lowWrite(WRITE_INTMSK,0b11110110);	// INTCON, enabled=0. RXIE and TXNIE only enabled.

	// Make RF communication stable under extreme temperatures
	highWrite(RFCTRL0, 0x03);			// this was previously done above
	highWrite(RFCTRL1, 0x02);			// VCCOPT - whatever that does...

	RadioSetChannel(RadioStatus.Channel);	// tune to current radio channel

	#ifdef TURBO_MODE					// propriatary TURBO_MODE runs at 625 kbps (vs. 802.15.4 compliant 250 kbps)
		lowWrite(WRITE_BBREG0, 0x01);	// TURBO mode enable
		lowWrite(WRITE_BBREG3, 0x38);	// PREVALIDTH to turbo optimized setting
		lowWrite(WRITE_BBREG4, 0x5C);	// CSTH carrier sense threshold to turbo optimal
	#endif

	lowWrite(WRITE_RFCTL,0x04);			// reset RF state machine
	lowWrite(WRITE_RFCTL,0x00);			// back to normal operation

	// now delay at least 192 uS per datasheet init



*/

}
