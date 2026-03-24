/*
 * NRF24 driver — directly copied from the NRF24 Jammer's proven library.
 * Key: uses device->cs_pin for chip select (NOT handle->cs).
 * This allows explicit pin assignment for Apex 5G boards (CS on PC3).
 * SPI bus is acquired once and held for entire session.
 */

#include "nrf24.h"
#include <furi.h>
#include <furi_hal.h>
#include <furi_hal_resources.h>
#include <string.h>
#include <storage/storage.h>
#include <flipper_format/flipper_format.h>

void nrf24_init(nrf24_device_t* dev) {
    if(dev->initialized) return;

    bool found_momentum = false;
    uint32_t spi_nrf24_handle = 0; // 0 = PA4 (Default), 1 = PC3 (Extra)
    
    Storage* storage = furi_record_open(RECORD_STORAGE);
    FlipperFormat* file = flipper_format_file_alloc(storage);
    if(flipper_format_file_open_existing(file, "/int/.momentum_settings.txt")) {
        if(flipper_format_read_uint32(file, "spi_nrf24_handle", &spi_nrf24_handle, 1)) {
            found_momentum = true;
        }
    }
    flipper_format_free(file);
    furi_record_close(RECORD_STORAGE);

    if(!found_momentum) {
        // If not momentum (or setting missing), default to pin extra 4 (PA4)
        dev->cs_pin = &gpio_ext_pa4;
    } else {
        if(spi_nrf24_handle == 1) { // SpiExtra (Pin 7 / PC3)
            dev->cs_pin = &gpio_ext_pc3;
        } else { // SpiDefault (Pin 4 / PA4)
            dev->cs_pin = &gpio_ext_pa4;
        }
    }

    furi_hal_gpio_init(dev->cs_pin, GpioModeOutputPushPull, GpioPullUp, GpioSpeedVeryHigh);
    furi_hal_gpio_write(dev->cs_pin, true);

    furi_hal_gpio_init(dev->ce_pin, GpioModeOutputPushPull, GpioPullUp, GpioSpeedVeryHigh);
    furi_hal_gpio_write(dev->ce_pin, false);

    furi_hal_spi_bus_handle_init(dev->spi_handle);
    furi_hal_spi_acquire(dev->spi_handle);

    /* --- HARDWARE CS COLLISION HACK ---
     * Flipper's `furi_hal_spi_bus_handle_external` uses `PA4` as its default CS.
     * When we call `furi_hal_spi_bus_trx()`, Furi HAL forces `PA4` low alongside `dev->cs_pin`.
     * If a CC1101 is on `PA4`, it wakes up and drives the MISO line at the exact same time
     * the NRF24 tries to respond! This destroys all SPI reads (returning 0x00 or garbage).
     * To prevent this, if we are NOT using PA4 as our CS pin (e.g. we use PC3), 
     * we intentionally reconfigure `PA4` as an INPUT with a pull-up resistor. 
     */
    if(dev->cs_pin != &gpio_ext_pa4) {
        furi_hal_gpio_init(&gpio_ext_pa4, GpioModeInput, GpioPullUp, GpioSpeedVeryHigh);
    }

    dev->initialized = true;
}

void nrf24_deinit(nrf24_device_t* dev) {
    if(!dev->initialized) return;

    furi_hal_gpio_write(dev->ce_pin, false);

    furi_hal_spi_release(dev->spi_handle);
    furi_hal_spi_bus_handle_deinit(dev->spi_handle);

    furi_hal_gpio_init(&gpio_ext_pa4, GpioModeAnalog, GpioPullNo, GpioSpeedLow);
    furi_hal_gpio_init(dev->ce_pin, GpioModeAnalog, GpioPullNo, GpioSpeedLow);
    furi_hal_gpio_init(dev->cs_pin, GpioModeAnalog, GpioPullNo, GpioSpeedLow);

    dev->initialized = false;
}

/* SPI transaction: toggle device->cs_pin (NOT handle->cs) */
void nrf24_spi_trx(
    nrf24_device_t* dev,
    uint8_t* tx,
    uint8_t* rx,
    uint8_t size,
    uint32_t timeout) {
    UNUSED(timeout);
    furi_hal_gpio_write(dev->cs_pin, false);
    furi_hal_spi_bus_trx(dev->spi_handle, tx, rx, size, nrf24_TIMEOUT);
    furi_hal_gpio_write(dev->cs_pin, true);
}

uint8_t nrf24_write_reg(nrf24_device_t* dev, uint8_t reg, uint8_t data) {
    uint8_t tx[2] = {W_REGISTER | (REGISTER_MASK & reg), data};
    uint8_t rx[2] = {0};
    nrf24_spi_trx(dev, tx, rx, 2, nrf24_TIMEOUT);
    return rx[0];
}

uint8_t nrf24_write_buf_reg(nrf24_device_t* dev, uint8_t reg, uint8_t* data, uint8_t size) {
    uint8_t tx[size + 1];
    uint8_t rx[size + 1];
    memset(rx, 0, size + 1);
    tx[0] = W_REGISTER | (REGISTER_MASK & reg);
    memcpy(&tx[1], data, size);
    nrf24_spi_trx(dev, tx, rx, size + 1, nrf24_TIMEOUT);
    return rx[0];
}

uint8_t nrf24_read_reg(nrf24_device_t* dev, uint8_t reg, uint8_t* data, uint8_t size) {
    uint8_t tx[size + 1];
    uint8_t rx[size + 1];
    memset(rx, 0, size + 1);
    tx[0] = R_REGISTER | (REGISTER_MASK & reg);
    memset(&tx[1], 0, size);
    nrf24_spi_trx(dev, tx, rx, size + 1, nrf24_TIMEOUT);
    memcpy(data, &rx[1], size);
    return rx[0];
}

uint8_t nrf24_status(nrf24_device_t* dev) {
    uint8_t status;
    uint8_t tx[] = {R_REGISTER | (REGISTER_MASK & REG_STATUS)};
    nrf24_spi_trx(dev, tx, &status, 1, nrf24_TIMEOUT);
    return status;
}

bool nrf24_check_connected(nrf24_device_t* dev) {
    uint8_t status = nrf24_status(dev);
    return (status != 0x00 && status != 0xFF);
}

uint8_t nrf24_flush_rx(nrf24_device_t* dev) {
    uint8_t tx[] = {FLUSH_RX};
    uint8_t rx[] = {0};
    nrf24_spi_trx(dev, tx, rx, 1, nrf24_TIMEOUT);
    return rx[0];
}

uint8_t nrf24_set_chan(nrf24_device_t* dev, uint8_t chan) {
    return nrf24_write_reg(dev, REG_RF_CH, chan);
}

uint8_t nrf24_set_idle(nrf24_device_t* dev) {
    uint8_t cfg = 0;
    nrf24_read_reg(dev, REG_CONFIG, &cfg, 1);
    cfg &= 0xfc;
    uint8_t status = nrf24_write_reg(dev, REG_CONFIG, cfg);
    furi_hal_gpio_write(dev->ce_pin, false);
    return status;
}

uint8_t nrf24_set_rx_mode(nrf24_device_t* dev) {
    uint8_t cfg = 0;
    nrf24_read_reg(dev, REG_CONFIG, &cfg, 1);
    cfg |= 0x03; /* PWR_UP + PRIM_RX */
    uint8_t status = nrf24_write_reg(dev, REG_CONFIG, cfg);
    furi_hal_gpio_write(dev->ce_pin, true);
    furi_delay_ms(2);
    return status;
}

uint8_t nrf24_power_up(nrf24_device_t* dev) {
    uint8_t cfg = 0;
    nrf24_read_reg(dev, REG_CONFIG, &cfg, 1);
    cfg |= 0x02;
    uint8_t status = nrf24_write_reg(dev, REG_CONFIG, cfg);
    furi_delay_ms(5);
    return status;
}
