# ESP32 SPI Master Test

This project demonstrates Master SPI communication on ESP32 microcontrollers using the Embassy async runtime in a `no_std` environment. It is designed to be only used with and to test the slave implementation provided on the other corresponding repo.

## Hardware Configuration

### Master Side Pin Configuration
- **SCLK** => GPIO27
- **MISO** => GPIO35 (input to ESP32) // the master input is not really used as communication is unidirectional
- **MOSI** => GPIO22 (output from ESP32)
- **CS** => GPIO21

### Pin Connections (feel free to change them)
| Signal | Master (ESP32) | Slave |
|--------|----------------|-------|
| SCLK   | GPIO27         | GPIO10|
| MISO   | GPIO35         | GPIO11|
| MOSI   | GPIO22         | GPIO12|
| CS     | GPIO21         | GPIO13|

You should also connect the ground (GND) between the master and slave devices to ensure a common reference point.

## Technical Specifications

- **SPI Configuration**: 60MHz, Mode 0 (configurable)
- **Buffer Size**: 2KB (configurable via `CHUNK_SIZE` const)
- **Tested Transfer Rate**: ~40Hz (~82KB/second) (can do more than that)
- **Operation**: Asynchronous master SPI chunk batch transfer

## Usage Notes

1. Ensure matching buffer sizes between master and slave devices
2. The program transmits at regular intervals with some simple data rate printing for debugging
3. Use the provided helper functions for transferring SPI, don't let the normal SPI driver provided by the HAL control the CS pin
4. Transfer is made by chunks, the bigger the chunks batch the better the performance should be.
5. Remember to change to your specific ESP32 version in the `Cargo.toml` file and config the build setup accordingly.

## Building and Running
This project uses the Embassy runtime framework and ESP32 HAL. Build with Rust using appropriate feature flags and toolchains (see https://docs.esp-rs.org/book/installation/riscv-and-xtensa.html for complete setup instructions). Flash to your ESP32 using standard ESP tools with `cargo run`.

Remember to change to your specific ESP32 version in the `Cargo.toml` file and config the build setup accordingly.

See the source code comments and the slave implementation docs for more information.

