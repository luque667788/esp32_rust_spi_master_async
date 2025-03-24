//! SPI Master Test with DMA
//!
//! Wiring connections:
//!
//! Slave side:
//! - SCLK => GPIO10
//! - MISO => GPIO11
//! - MOSI => GPIO12
//! - CS   => GPIO13
//!
//! Connection mapping:
//! - 13 -> 21 (CS)
//! - 12 -> 22 (MOSI)
//! - 11 -> 35 (MISO)
//! - 10 -> 27 (SCLK)
//!
//! Master side:
//! - SCLK => GPIO27
//! - MISO => GPIO35
//! - MOSI => GPIO22
//! - CS   => GPIO21
//!
//! This example demonstrates SPI communication using DMA with the ESP32.
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer, Instant};
use embedded_hal_async::spi::SpiBus;
use esp_backtrace as _;
use esp_hal::{
    gpio::{Level, Output, OutputConfig},
    spi::{master::Spi, Error as SpiError},
    timer::timg::TimerGroup,
};
use esp_println::println;

// Configuration constants
const CHUNK_SIZE: usize = 512 * 4; // 2KB buffer size 512 floats chunksize -> you can change this of course
// make sure the CHUNK_SIZE in the client code is the same also
// to avoid buffer overflows (supposely the esp-hal dma has its own SPI buffer but i asked in their matrix forum and it is not working reliably as i tested it also)

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
   // Initialize peripherals
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // Set up embassy time source
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    // Configure GPIO for chip select
    let mut cs_pin = Output::new(peripherals.GPIO21, Level::High, OutputConfig::default());

    // Start background monitoring task
    spawner.spawn(background_task()).unwrap();

    // Configure SPI master
    let mut spi_master = Spi::new(
        peripherals.SPI3,
        esp_hal::spi::master::Config::default()
            .with_frequency(esp_hal::time::Rate::from_mhz(60))
            .with_mode(esp_hal::spi::Mode::_0),
    )
    .unwrap()
    .with_sck(peripherals.GPIO27)
    .with_mosi(peripherals.GPIO22)
    .with_miso(peripherals.GPIO35) // CS pin handled manually
    .into_async();

    let mut tx_buffer = [0u8; CHUNK_SIZE];
    let mut total_data_sent = 0;
    let mut start_time = Instant::now();
    let mut last_elapsed_time = 0_u64;

    tx_buffer[2] = 0xFF;// just some marker to check in the slave if we are getting the right data
    // Main  loop
    loop {

        // Mark current position in buffer (just for testing generating some data and the sending it)
        

        // Send data via SPI
        transmit_data(&mut spi_master, &tx_buffer, &mut cs_pin)
            .await
            .unwrap();


        tx_buffer[2] = 0xFF;
        total_data_sent += tx_buffer.len();



        // Log transmitted data -> comment this out because at this data rate it will be too much prints
        // if tx_buffer.len() > 20 {
        //     println!(
        //         "Transmitted: beginning={:x?} ... end={:x?}",
        //         &tx_buffer[..10],
        //         &tx_buffer[tx_buffer.len() - 10..],
        //     );
        // } else {
        //     println!("Transmitted: {:x?}", &tx_buffer);
        // }
        
        Timer::after(Duration::from_millis(20)).await; // Wait for 20ms before sending the next chunk, so we roughly aproach 40Hz and the 82 kilobytes per second

        // Calculate and print data rate every second
        let elapsed_time = start_time.elapsed().as_secs();
        if elapsed_time > 0 && elapsed_time % 10 == 0 && last_elapsed_time != elapsed_time {
            
            let data_rate = total_data_sent as f64 / (start_time.elapsed().as_millis() as f64 / 1000.0);
            println!("Total data sent: {} bytes", total_data_sent);
            println!("Data rate: {:.2} bytes/second", data_rate);
            start_time = Instant::now(); // Reset start time
            total_data_sent = 0; // Reset total data sent
            // Reset elapsed time
            last_elapsed_time = elapsed_time;

            tx_buffer[2] = 0xED;
        }
    }
}

/// Transmits data over SPI with proper CS handling
async fn transmit_data(
    spi: &mut Spi<'_, esp_hal::Async>,
    data: &[u8],
    cs: &mut Output<'_>,
) -> Result<(), SpiError> {
    //println!("Beginning SPI transmission (CS active)");
    cs.set_low(); // Activate chip select
    SpiBus::write(spi, data).await?;//non blocking write
    cs.set_high(); // Deactivate chip select
    Ok(())
}

/// Background monitoring task
#[embassy_executor::task]
async fn background_task() {
    loop {
        Timer::after(Duration::from_millis(1000)).await;
        println!("Background monitor: system active");
    }
}
