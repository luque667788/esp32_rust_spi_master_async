# ESP32 SPI Master Test

This project demonstrates SPI communication on ESP32 microcontrollers using the Embassy async runtime in a `no_std` environment. It is designed to be used with and test the slave implementation provided.

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

- **SPI Configuration**: 60MHz, Mode 0
- **Buffer Size**: 2KB (configurable via `CHUNK_SIZE` const)
- **Tested Transfer Rate**: ~40Hz (~82KB/second) (can do more than that)
- **Operation**: Asynchronous master SPI chunk batch transfer

## Usage Notes

1. Ensure matching buffer sizes between master and slave devices
2. The program transmits at regular intervals with some simple data rate printing for debugging
3. More debug output can be enabled by uncommenting print statements
4. Use the provided helper functions for transferring SPI, don't let the normal SPI driver provided by the HAL control the CS pin
5. Transfer is made by chunks, the bigger the chunks batch the better the performance should be.
6. Remember to change to your specific ESP32 version in the `Cargo.toml` file and config the build setup accordingly.

## Building the Project

- To build:
```bash
cargo build
```

- To run:
```bash
cargo run
```
See the source code comments and the slave implementation docs for more information.


Feel free to contact me if you have any issues, doubts, or questions!
