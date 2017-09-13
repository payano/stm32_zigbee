/*
 * Mrf24j.cpp
 *
 *  Created on: Jul 6, 2017
 *      Author: johan
 */

#include "Mrf24j.h"

#define MRF_RXMCR 0x00
#define MRF_PANIDL 0x01
#define MRF_PANIDH 0x02
#define MRF_SADRL 0x03
#define MRF_SADRH 0x04
#define MRF_EADR0 0x05
#define MRF_EADR1 0x06
#define MRF_EADR2 0x07
#define MRF_EADR3 0x08
#define MRF_EADR4 0x09
#define MRF_EADR5 0x0A
#define MRF_EADR6 0x0B
#define MRF_EADR7 0x0C
#define MRF_RXFLUSH 0x0D
//#define MRF_Reserved 0x0E
//#define MRF_Reserved 0x0F
#define MRF_ORDER 0x10
#define MRF_TXMCR 0x11
#define MRF_ACKTMOUT 0x12
#define MRF_ESLOTG1 0x13
#define MRF_SYMTICKL 0x14
#define MRF_SYMTICKH 0x15
#define MRF_PACON0 0x16
#define MRF_PACON1 0x17
#define MRF_PACON2 0x18
//#define MRF_Reserved 0x19
#define MRF_TXBCON0 0x1A

// TXNCON: TRANSMIT NORMAL FIFO CONTROL REGISTER (ADDRESS: 0x1B)
#define MRF_TXNCON      0x1B
#define MRF_TXNTRIG     0
#define MRF_TXNSECEN    1
#define MRF_TXNACKREQ   2
#define MRF_INDIRECT    3
#define MRF_FPSTAT      4

#define MRF_TXG1CON 0x1C
#define MRF_TXG2CON 0x1D
#define MRF_ESLOTG23 0x1E
#define MRF_ESLOTG45 0x1F
#define MRF_ESLOTG67 0x20
#define MRF_TXPEND 0x21
#define MRF_WAKECON 0x22
#define MRF_FRMOFFSET 0x23
// TXSTAT: TX MAC STATUS REGISTER (ADDRESS: 0x24)
#define MRF_TXSTAT 0x24
#define TXNRETRY1       7
#define TXNRETRY0       6
#define CCAFAIL         5
#define TXG2FNT         4
#define TXG1FNT         3
#define TXG2STAT        2
#define TXG1STAT        1
#define TXNSTAT         0

#define MRF_TXBCON1 0x25
#define MRF_GATECLK 0x26
#define MRF_TXTIME 0x27
#define MRF_HSYMTMRL 0x28
#define MRF_HSYMTMRH 0x29
#define MRF_SOFTRST 0x2A
//#define MRF_Reserved 0x2B
#define MRF_SECCON0 0x2C
#define MRF_SECCON1 0x2D
#define MRF_TXSTBL 0x2E
//#define MRF_Reserved 0x2F
#define MRF_RXSR 0x30
#define MRF_INTSTAT 0x31
#define MRF_INTCON 0x32
#define MRF_GPIO 0x33
#define MRF_TRISGPIO 0x34
#define MRF_SLPACK 0x35
#define MRF_RFCTL 0x36
#define MRF_SECCR2 0x37
#define MRF_BBREG0 0x38
#define MRF_BBREG1 0x39
#define MRF_BBREG2 0x3A
#define MRF_BBREG3 0x3B
#define MRF_BBREG4 0x3C
//#define MRF_Reserved 0x3D
#define MRF_BBREG6 0x3E
#define MRF_CCAEDTH 0x3F

#define MRF_RFCON0 0x200
#define MRF_RFCON1 0x201
#define MRF_RFCON2 0x202
#define MRF_RFCON3 0x203
#define MRF_RFCON5 0x205
#define MRF_RFCON6 0x206
#define MRF_RFCON7 0x207
#define MRF_RFCON8 0x208
#define MRF_SLPCAL0 0x209
#define MRF_SLPCAL1 0x20A
#define MRF_SLPCAL2 0x20B
#define MRF_RSSI 0x210
#define MRF_SLPCON0 0x211
#define MRF_SLPCON1 0x220
#define MRF_WAKETIMEL 0x222
#define MRF_WAKETIMEH 0x223
#define MRF_REMCNTL 0x224
#define MRF_REMCNTH 0x225
#define MRF_MAINCNT0 0x226
#define MRF_MAINCNT1 0x227
#define MRF_MAINCNT2 0x228
#define MRF_MAINCNT3 0x229
#define MRF_TESTMODE 0x22F
#define MRF_ASSOEADR1 0x231
#define MRF_ASSOEADR2 0x232
#define MRF_ASSOEADR3 0x233
#define MRF_ASSOEADR4 0x234
#define MRF_ASSOEADR5 0x235
#define MRF_ASSOEADR6 0x236
#define MRF_ASSOEADR7 0x237
#define MRF_ASSOSADR0 0x238
#define MRF_ASSOSADR1 0x239
#define MRF_UPNONCE0 0x240
#define MRF_UPNONCE1 0x241
#define MRF_UPNONCE2 0x242
#define MRF_UPNONCE3 0x243
#define MRF_UPNONCE4 0x244
#define MRF_UPNONCE5 0x245
#define MRF_UPNONCE6 0x246
#define MRF_UPNONCE7 0x247
#define MRF_UPNONCE8 0x248
#define MRF_UPNONCE9 0x249
#define MRF_UPNONCE10 0x24A
#define MRF_UPNONCE11 0x24B
#define MRF_UPNONCE12 0x24C

#define MRF_I_RXIF  0b00001000
#define MRF_I_TXNIF 0b00000001

// aMaxPHYPacketSize = 127, from the 802.15.4-2006 standard.
static uint8_t rx_buf[127];

// essential for obtaining the data frame only
// bytes_MHR = 2 Frame control + 1 sequence number + 2 panid + 2 shortAddr Destination + 2 shortAddr Source
static int bytes_MHR = 9;
static int bytes_FCS = 2; // FCS length = 2
static int bytes_nodata = bytes_MHR + bytes_FCS; // no_data bytes in PHY payload,  header length + FCS

static int ignoreBytes = 0; // bytes to ignore, some modules behaviour.

static bool bufPHY = false; // flag to buffer all bytes in PHY Payload, or not

volatile uint8_t flag_got_rx;
volatile uint8_t flag_got_tx;

static rx_info_t rx_info;
static tx_info_t tx_info;


Mrf24j::Mrf24j(
		SPI_HandleTypeDef* spi_handler,
		pinIO& pinReset,
		pinIO& pinCs,
		pinIO& pinInterrupt):
		mSpi_handler(spi_handler),
		mPinReset(pinReset),
		mPinCs(pinCs),
		mPinInterrupt(pinInterrupt)
{

}

Mrf24j::~Mrf24j() {
	// TODO Auto-generated destructor stub
}
void Mrf24j::run(){

//	while(true){
		char kalle[] = "abcd";
		send16(0x4202, kalle);
		osDelay(1000);
//	}

}

void Mrf24j::int_callback(){
	mCallback = true;
}

void Mrf24j::reset(void){
	//https://github.com/karlp/Mrf24j40-arduino-library/blob/master/mrf24j.cpp#L1
	HAL_GPIO_WritePin(mPinReset.GPIO, mPinReset.GPIO_Pin, GPIO_PIN_RESET);
	osDelay(10);
	HAL_GPIO_WritePin(mPinReset.GPIO, mPinReset.GPIO_Pin, GPIO_PIN_SET);
	osDelay(20);
}

void Mrf24j::init(void) {
    /*
    // Seems a bit ridiculous when I use reset pin anyway
    write_short(MRF_SOFTRST, 0x7); // from manual
    while (read_short(MRF_SOFTRST) & 0x7 != 0) {
        ; // wait for soft reset to finish
    }
    */
    write_short(MRF_PACON2, 0x98); // – Initialize FIFOEN = 1 and TXONTS = 0x6.
    write_short(MRF_TXSTBL, 0x95); // – Initialize RFSTBL = 0x9.

    write_long(MRF_RFCON0, 0x03); // – Initialize RFOPT = 0x03.
    write_long(MRF_RFCON1, 0x01); // – Initialize VCOOPT = 0x02.
    write_long(MRF_RFCON2, 0x80); // – Enable PLL (PLLEN = 1).
    write_long(MRF_RFCON6, 0x90); // – Initialize TXFIL = 1 and 20MRECVR = 1.
    write_long(MRF_RFCON7, 0x80); // – Initialize SLPCLKSEL = 0x2 (100 kHz Internal oscillator).
    write_long(MRF_RFCON8, 0x10); // – Initialize RFVCO = 1.
    write_long(MRF_SLPCON1, 0x21); // – Initialize CLKOUTEN = 1 and SLPCLKDIV = 0x01.

    //  Configuration for nonbeacon-enabled devices (see Section 3.8 “Beacon-Enabled and
    //  Nonbeacon-Enabled Networks”):
    write_short(MRF_BBREG2, 0x80); // Set CCA mode to ED
    write_short(MRF_CCAEDTH, 0x60); // – Set CCA ED threshold.
    write_short(MRF_BBREG6, 0x40); // – Set appended RSSI value to RXFIFO.
    set_interrupts();
    set_channel(12);
    // max power is by default.. just leave it...
    // Set transmitter power - See “REGISTER 2-62: RF CONTROL 3 REGISTER (ADDRESS: 0x203)”.
    write_short(MRF_RFCTL, 0x04); //  – Reset RF state machine.
    write_short(MRF_RFCTL, 0x00); // part 2
    osDelay(1); // delay at least 192usec
}
uint16_t Mrf24j::get_pan(void) {
    uint8_t panh = read_short(MRF_PANIDH);
    return panh << 8 | read_short(MRF_PANIDL);
}

void Mrf24j::set_pan(uint16_t panid) {
    write_short(MRF_PANIDH, panid >> 8);
    write_short(MRF_PANIDL, panid & 0xff);
}

void Mrf24j::address16_write(uint16_t address16) {
    write_short(MRF_SADRH, address16 >> 8);
    write_short(MRF_SADRL, address16 & 0xff);
}

uint16_t Mrf24j::address16_read(void) {
    uint8_t a16h = read_short(MRF_SADRH);
    return a16h << 8 | read_short(MRF_SADRL);
}

void Mrf24j::set_interrupts(void) {
    // interrupts for rx and tx normal complete
    write_short(MRF_INTCON, 0b11110110);
}

/** use the 802.15.4 channel numbers..
 */
void Mrf24j::set_channel(uint8_t channel) {
    write_long(MRF_RFCON0, (((channel - 11) << 4) | 0x03));
    read_long(MRF_RFCON0);
}

/**
 * Simple send 16, with acks, not much of anything.. assumes src16 and local pan only.
 * @param data
 */
void Mrf24j::send16(uint16_t dest16, char * data) {
    uint8_t len = strlen(data); // get the length of the char* array
    int i = 0;
    write_long(i++, bytes_MHR); // header length
    // +ignoreBytes is because some module seems to ignore 2 bytes after the header?!.
    // default: ignoreBytes = 0;
    write_long(i++, bytes_MHR+ignoreBytes+len);

    // 0 | pan compression | ack | no security | no data pending | data frame[3 bits]
    write_long(i++, 0b01100001); // first byte of Frame Control
    // 16 bit source, 802.15.4 (2003), 16 bit dest,
    write_long(i++, 0b10001000); // second byte of frame control
    write_long(i++, 1);  // sequence number 1

    uint16_t panid = get_pan();

    write_long(i++, panid & 0xff);  // dest panid
    write_long(i++, panid >> 8);
    write_long(i++, dest16 & 0xff);  // dest16 low
    write_long(i++, dest16 >> 8); // dest16 high

    uint16_t src16 = address16_read();
    write_long(i++, src16 & 0xff); // src16 low
    write_long(i++, src16 >> 8); // src16 high

    // All testing seems to indicate that the next two bytes are ignored.
    //2 bytes on FCS appended by TXMAC
    i+=ignoreBytes;
    for (int q = 0; q < len; q++) {
        write_long(i++, data[q]);
    }
    // ack on, and go!
    write_short(MRF_TXNCON, (1<<MRF_TXNACKREQ | 1<<MRF_TXNTRIG));
}

/**
 * Call this from within an interrupt handler connected to the MRFs output
 * interrupt pin.  It handles reading in any data from the module, and letting it
 * continue working.
 * Only the most recent data is ever kept.
 */
void Mrf24j::interrupt_handler(void) {
    uint8_t last_interrupt = read_short(MRF_INTSTAT);
    if (last_interrupt & MRF_I_RXIF) {
        flag_got_rx++;
        // read out the packet data...
        //TODO FIX THIS
        //noInterrupts();
        //TODO FIX PROPER HANDLER FOR INTERRUPT
        interrupt(false);

        rx_disable();
        // read start of rxfifo for, has 2 bytes more added by FCS. frame_length = m + n + 2
        uint8_t frame_length = read_long(0x300);

        // buffer all bytes in PHY Payload
        if(bufPHY){
            int rb_ptr = 0;
            for (int i = 0; i < frame_length; i++) { // from 0x301 to (0x301 + frame_length -1)
                rx_buf[rb_ptr++] = read_long(0x301 + i);
            }
        }

        // buffer data bytes
        int rd_ptr = 0;
        // from (0x301 + bytes_MHR) to (0x301 + frame_length - bytes_nodata - 1)
        for (int i = 0; i < rx_datalength(); i++) {
            rx_info.rx_data[rd_ptr++] = read_long(0x301 + bytes_MHR + i);
        }

        rx_info.frame_length = frame_length;
        // same as datasheet 0x301 + (m + n + 2) <-- frame_length
        rx_info.lqi = read_long(0x301 + frame_length);
        // same as datasheet 0x301 + (m + n + 3) <-- frame_length + 1
        rx_info.rssi = read_long(0x301 + frame_length + 1);

        rx_enable();

        //TODO FIX THIS
        interrupt(true);
        //interrupts();
    }
    if (last_interrupt & MRF_I_TXNIF) {
        flag_got_tx++;
        uint8_t tmp = read_short(MRF_TXSTAT);
        // 1 means it failed, we want 1 to mean it worked.
        tx_info.tx_ok = !(tmp & ~(1 << TXNSTAT));
        tx_info.retries = tmp >> 6;
        tx_info.channel_busy = (tmp & (1 << CCAFAIL));
    }
}
/**
 * Set RX mode to promiscuous, or normal
 */
void Mrf24j::set_promiscuous(bool enabled) {
    if (enabled) {
        write_short(MRF_RXMCR, 0x01);
    } else {
        write_short(MRF_RXMCR, 0x00);
    }
}

rx_info_t * Mrf24j::get_rxinfo(void) {
    return &rx_info;
}

tx_info_t * Mrf24j::get_txinfo(void) {
    return &tx_info;
}

uint8_t * Mrf24j::get_rxbuf(void) {
    return rx_buf;
}

int Mrf24j::rx_datalength(void) {
    return rx_info.frame_length - bytes_nodata;
}

void Mrf24j::set_ignoreBytes(int ib) {
    // some modules behaviour
    ignoreBytes = ib;
}

/**
 * Set bufPHY flag to buffer all bytes in PHY Payload, or not
 */
void Mrf24j::set_bufferPHY(bool bp) {
    bufPHY = bp;
}

bool Mrf24j::get_bufferPHY(void) {
    return bufPHY;
}

/**
 * Set PA/LNA external control
 */
void Mrf24j::set_palna(bool enabled) {
    if (enabled) {
        write_long(MRF_TESTMODE, 0x07); // Enable PA/LNA on MRF24J40MB module.
    }else{
        write_long(MRF_TESTMODE, 0x00); // Disable PA/LNA on MRF24J40MB module.
    }
}

void Mrf24j::rx_flush(void) {
    write_short(MRF_RXFLUSH, 0x01);
}

void Mrf24j::rx_disable(void) {
    write_short(MRF_BBREG1, 0x04);  // RXDECINV - disable receiver
}

void Mrf24j::rx_enable(void) {
    write_short(MRF_BBREG1, 0x00);  // RXDECINV - enable receiver
}
/**
 * Call this function periodically, it will invoke your nominated handlers
 */
void Mrf24j::check_flags(void (*rx_handler)(void), void (*tx_handler)(void)){
    // TODO - we could check whether the flags are > 1 here, indicating data was lost?
    if (flag_got_rx) {
        flag_got_rx = 0;
        rx_handler();
    }
    if (flag_got_tx) {
        flag_got_tx = 0;
        tx_handler();
    }
}
uint8_t Mrf24j::read_short(uint8_t address){
	uint8_t result_data;
	//Mask data correctly before sending.
	address = address << 1;
	address &= 0b01111110;

	//Enable CS Pin, make MRF aware of incoming message.
	HAL_GPIO_WritePin(mPinCs.GPIO, mPinCs.GPIO_Pin, GPIO_PIN_RESET);

	//Send address and receive data
	HAL_SPI_Transmit(mSpi_handler,&address, bufferSize, 1);
	HAL_SPI_Receive(mSpi_handler,&result_data,bufferSize, 1);

	//Disable transfer to mrf.
	HAL_GPIO_WritePin(mPinCs.GPIO, mPinCs.GPIO_Pin, GPIO_PIN_SET);

	return result_data;
}
uint8_t Mrf24j::read_long(uint16_t address){
	uint8_t result_data;

	//Mask data correctly before reading.
	uint8_t address1 = ((address>>3)&0x7F)|0x80;
	uint8_t address2 = ((address<<5)&0xE0);

	//Enable CS Pin, make MRF aware of incoming message.
	HAL_GPIO_WritePin(mPinCs.GPIO, mPinCs.GPIO_Pin, GPIO_PIN_RESET);

	//Send address and receieve data
	HAL_SPI_Transmit(mSpi_handler,&address1, 1, 1);
	HAL_SPI_Transmit(mSpi_handler, &address2, 1, 1);
	HAL_SPI_Receive(mSpi_handler, &result_data, 1, 1);

	//Disable transfer to mrf.
	HAL_GPIO_WritePin(mPinCs.GPIO, mPinCs.GPIO_Pin, GPIO_PIN_SET);

	return result_data;
}
void Mrf24j::write_short(uint8_t address, uint8_t data){
	//Mask data correctly before sending.
	address = address << 1;
	address &= 0b01111111;
	address |= 0x01;

	//Enable CS Pin, make MRF aware of incoming message.
	HAL_GPIO_WritePin(mPinCs.GPIO, mPinCs.GPIO_Pin, GPIO_PIN_RESET);

	//Send address and data
	HAL_SPI_Transmit(mSpi_handler,&address, 1, 1);
	HAL_SPI_Transmit(mSpi_handler,&data, 1, 1);

	//Disable transfer to mrf.
	HAL_GPIO_WritePin(mPinCs.GPIO, mPinCs.GPIO_Pin, GPIO_PIN_SET);
}
void Mrf24j::write_long(uint16_t address, uint8_t data)
{
	//Mask data correctly before sending.
	uint8_t address1 = (((uint8_t)(address>>3))&0x7F)|0x80;
	uint8_t address2 = (((uint8_t)(address<<5))&0xE0)|0x10;

	//Enable CS Pin, make MRF aware of incoming message.
	HAL_GPIO_WritePin(mPinCs.GPIO, mPinCs.GPIO_Pin, GPIO_PIN_RESET);

	//Send address and data
	HAL_SPI_Transmit(mSpi_handler, &address1, 1, 1);
	HAL_SPI_Transmit(mSpi_handler, &address2, 1, 1);
	HAL_SPI_Transmit(mSpi_handler, &data, 1, 1);

	//Disable transfer to mrf.
	HAL_GPIO_WritePin(mPinCs.GPIO, mPinCs.GPIO_Pin, GPIO_PIN_SET);
}

void Mrf24j::interrupt(bool value){
	//Todo fix this handler to be better implemented.
	switch(value){
	case true:
		HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
		break;
	default:
		HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
		break;
	}
}
