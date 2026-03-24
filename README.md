# flipper-nrf-spectrum

NRF24 spectrum analyzer application for the Flipper Zero.

## Firmware Compatibility
- **Momentum** / **Unleashed**: Supported. App dynamically detects custom NRF SPI CS pin configuration via OS settings.
- **Stock / Other**: Supported. Defaults entirely to Extra 4 (PA4).

## Supported Hardware
- Standard NRF24 modules.
- Extensively tested on **Apex 5G Marauder** boards from **honeyhoney labs**.

## Technical Details
- Performs dynamic lookup of `/int/.momentum_settings.txt` for `spi_nrf24_handle`.
- If PC3 (`SpiExtra`, Pin 7) is detected as the selected NRF chip select, PA4 (`SpiDefault`) is forcefully disabled/sabotaged to prevent SPI collision with secondary CC1101 modules.