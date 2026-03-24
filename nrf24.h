#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <furi_hal_spi.h>

/*
 * NRF24 driver — copied from the NRF24 Jammer's proven library.
 * Uses nrf24_device_t with explicit cs_pin for Apex 5G support.
 */

#define R_REGISTER 0x00
#define W_REGISTER 0x20
#define REGISTER_MASK 0x1F
#define R_RX_PAYLOAD 0x61
#define W_TX_PAYLOAD 0xA0
#define W_TX_PAYLOAD_NOACK 0xB0
#define FLUSH_TX 0xE1
#define FLUSH_RX 0xE2
#define RF24_NOP 0xFF

#define REG_CONFIG      0x00
#define REG_EN_AA       0x01
#define REG_EN_RXADDR   0x02
#define REG_SETUP_AW    0x03
#define REG_SETUP_RETR  0x04
#define REG_RF_CH       0x05
#define REG_RF_SETUP    0x06
#define REG_STATUS      0x07
#define REG_RPD         0x09
#define REG_RX_ADDR_P0  0x0A
#define REG_TX_ADDR     0x10
#define RX_PW_P0        0x11
#define REG_FIFO_STATUS 0x17
#define REG_DYNPD       0x1C
#define REG_FEATURE     0x1D

#define RX_DR    0x40
#define TX_DS    0x20
#define MAX_RT   0x10

#define nrf24_TIMEOUT 500

typedef struct {
    FuriHalSpiBusHandle* spi_handle;
    const GpioPin* ce_pin;
    const GpioPin* cs_pin;
    bool initialized;
} nrf24_device_t;

void nrf24_init(nrf24_device_t* dev);
void nrf24_deinit(nrf24_device_t* dev);
bool nrf24_check_connected(nrf24_device_t* dev);

void nrf24_spi_trx(nrf24_device_t* dev, uint8_t* tx, uint8_t* rx, uint8_t size, uint32_t timeout);
uint8_t nrf24_write_reg(nrf24_device_t* dev, uint8_t reg, uint8_t data);
uint8_t nrf24_write_buf_reg(nrf24_device_t* dev, uint8_t reg, uint8_t* data, uint8_t size);
uint8_t nrf24_read_reg(nrf24_device_t* dev, uint8_t reg, uint8_t* data, uint8_t size);
uint8_t nrf24_status(nrf24_device_t* dev);
uint8_t nrf24_flush_rx(nrf24_device_t* dev);
uint8_t nrf24_set_chan(nrf24_device_t* dev, uint8_t chan);
uint8_t nrf24_set_idle(nrf24_device_t* dev);
uint8_t nrf24_set_rx_mode(nrf24_device_t* dev);
uint8_t nrf24_power_up(nrf24_device_t* dev);
