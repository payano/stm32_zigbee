/*
 * Mrf24j.h
	Inspired from: https://github.com/karlp/Mrf24j40-arduino-library/blob/master/mrf24j.h
	Datasheet: http://ww1.microchip.com/downloads/en/DeviceDoc/39776C.pdf
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

typedef struct _rx_info_t{
	uint8_t frame_length;
    uint8_t rx_data[116];
    /*  max data length = (127 aMaxPHYPacketSize,
    - 2 Frame control
    - 1 sequence number
    - 2 panid
    - 2 shortAddr Destination
    - 2 shortAddr Source
    - 2 FCS)
    */
    uint8_t lqi;
    uint8_t rssi;
} rx_info_t;

/**
 * Based on the TXSTAT register, but "better"
 */
typedef struct _tx_info_t{
    uint8_t tx_ok:1;
    uint8_t retries:2;
    uint8_t channel_busy:1;
} tx_info_t;

struct pinIO{
	GPIO_TypeDef* GPIO;
	uint16_t GPIO_Pin;
};
typedef struct pinIO pinIO;

class Mrf24j {
public:
    void check_flags(void (*rx_handler)(void), void (*tx_handler)(void));
	Mrf24j(SPI_HandleTypeDef* spi_handler, std::unique_ptr<pinIO> pinReset, std::unique_ptr<pinIO> pinCs, std::unique_ptr<pinIO> pinInterrupt);
	Mrf24j(SPI_HandleTypeDef* spi_handler, std::unique_ptr<pinIO> pinReset);
	virtual ~Mrf24j();
	virtual void run();
	virtual void int_callback();

    //Mrf24j(int pin_reset, int pin_chip_select, int pin_interrupt);
    void reset(void);
    void init(void);

    uint16_t get_pan(void);
    void set_pan(uint16_t panid);

    void address16_write(uint16_t address16);
    uint16_t address16_read(void);

    void set_interrupts(void);

    void set_promiscuous(bool enabled);

    /**
     * Set the channel, using 802.15.4 channel numbers (11..26)
     */
    void set_channel(uint8_t channel);

    void rx_enable(void);
    void rx_disable(void);

    /** If you want to throw away rx data */
    void rx_flush(void);

    rx_info_t* get_rxinfo(void);

    tx_info_t* get_txinfo(void);

    uint8_t* get_rxbuf(void);

    int rx_datalength(void);

    void set_ignoreBytes(int ib);

    /**
     * Set bufPHY flag to buffer all bytes in PHY Payload, or not
     */
    void set_bufferPHY(bool bp);

    bool get_bufferPHY(void);

    /**
     * Set PA/LNA external control
     */
    void set_palna(bool enabled);

    void send16(uint16_t dest16, char * data);

    void interrupt_handler(void);

private:
    //Need more to set the pin on the STM32 than ints..
	/*int mPin_reset;
    int mPin_cs;
    int mPin_int;*/
    uint8_t read_short(uint8_t address);
    uint8_t read_long(uint16_t address);

    void write_short(uint8_t address, uint8_t data);
    void write_long(uint16_t address, uint8_t data);

	void sendData(uint8_t address, uint8_t value);
	void recvData(uint8_t address, uint8_t* result);
	void sendLongData(uint16_t address, uint8_t value);
	void readLongData(uint16_t address, uint8_t* result);

	//Data members
	SPI_HandleTypeDef *mSpi_handler;
	const std::unique_ptr<pinIO> mPinReset, mPinCs, mPinInterrupt;
	bool mCallback = false;
    //might not be const...
	//pinIO &mReset, &mCs, &mPin_interrupt;
	uint8_t const bufferSize = 1;
	const uint8_t delay = 50;

};

#endif /* MRF24J_H_ */
