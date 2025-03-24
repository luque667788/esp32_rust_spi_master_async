//! SPI Master Test
//!
//! Wiring connections:
//!
//! Master side:
//! - SCLK => GPIO27
//! - MISO => GPIO35
//! - MOSI => GPIO22
//! - CS   => GPIO21
//!

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use embedded_hal_async::spi::SpiBus;
use esp_backtrace as _;
use esp_hal::{
    gpio::{Level, Output, OutputConfig},
    spi::{master::Spi, Error as SpiError},
    timer::timg::TimerGroup,
};
use esp_println::println;

// Configuration constants
const CHUNK_SIZE: usize = 512 * 4; // 2KB buffer size (512 float values)
                                   // Ensure the CHUNK_SIZE matches the client code configuration
                                   // to prevent buffer overflow issues. 

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // Initialize device peripherals
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // Configure Embassy time source
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    // Configure chip select GPIO pin with initial high state (inactive)
    let mut cs_pin = Output::new(peripherals.GPIO21, Level::High, OutputConfig::default());

    // Launch system monitoring background task
    spawner.spawn(background_task()).unwrap();

    // Configure SPI master with 60MHz clock in Mode 0
    let mut spi_master = Spi::new(
        peripherals.SPI3,
        esp_hal::spi::master::Config::default()
            .with_frequency(esp_hal::time::Rate::from_mhz(60))
            .with_mode(esp_hal::spi::Mode::_0),
    )
    .unwrap()
    .with_sck(peripherals.GPIO27)
    .with_mosi(peripherals.GPIO22)
    .with_miso(peripherals.GPIO35) // CS pin managed separately
    .into_async();

    let mut tx_buffer = [0u8; CHUNK_SIZE];
    let mut total_data_sent = 0;
    let mut start_time = Instant::now();
    let mut last_elapsed_time = 0_u64;

    tx_buffer[2] = 0xFF; // Marker byte to verify correct data reception at the slave

    // Main transmission loop
    loop {
        // Transmit data chunk via SPI
        transmit_data(&mut spi_master, &tx_buffer, &mut cs_pin)
            .await
            .unwrap();

        tx_buffer[2] = 0xFF;
        total_data_sent += tx_buffer.len();

        // Debug data content - disabled during normal operation
        // Output would be excessive at high data rates
        // if tx_buffer.len() > 20 {
        //     println!(
        //         "Transmitted: beginning={:x?} ... end={:x?}",
        //         &tx_buffer[..10],
        //         &tx_buffer[tx_buffer.len() - 10..],
        //     );
        // } else {
        //     println!("Transmitted: {:x?}", &tx_buffer);
        // }

        // Wait 20ms between transmissions to achieve ~40Hz rate
        // and approximately 82KB/sec throughput
        Timer::after(Duration::from_millis(20)).await;

        // Calculate and report performance metrics every 10 seconds
        let elapsed_time = start_time.elapsed().as_secs();
        if elapsed_time > 0 && elapsed_time % 10 == 0 && last_elapsed_time != elapsed_time {
            let data_rate =
                total_data_sent as f64 / (start_time.elapsed().as_millis() as f64 / 1000.0);
            println!("Total data sent: {} bytes", total_data_sent);
            println!("Data rate: {:.2} bytes/second", data_rate);
            start_time = Instant::now(); // Reset timing reference
            total_data_sent = 0;         // Reset data counter
            last_elapsed_time = elapsed_time;

            tx_buffer[2] = 0xED; // Update marker byte after statistics reset
        }
    }
}

/// Handles SPI data transmission with proper chip select signaling protocol
///
/// This function asserts the chip select (CS) pin to initiate communication, performs
/// a non-blocking data transfer via SPI, and then releases the CS pin.
///
/// # Arguments
///
/// * `spi` - A mutable reference to the asynchronous SPI master.
/// * `data` - A slice of bytes to be transmitted.
/// * `cs` - A mutable reference to the chip select GPIO pin.
///
/// # Returns
///
/// * `Result<(), SpiError>` - Returns `Ok(())` if the transmission is successful, or an `SpiError` if it fails.
async fn transmit_data(
    spi: &mut Spi<'_, esp_hal::Async>,
    data: &[u8],
    cs: &mut Output<'_>,
) -> Result<(), SpiError> {
    // Assertion of chip select (active low)
    cs.set_low();
    
    // Execute non-blocking data transfer
    SpiBus::write(spi, data).await?;
    
    // Release chip select
    cs.set_high();
    
    Ok(())
}

/// System monitoring task that runs concurrently with main operation
#[embassy_executor::task]
async fn background_task() {
    loop {
        Timer::after(Duration::from_millis(1000)).await;
        println!("Background monitor: system active");
    }
}