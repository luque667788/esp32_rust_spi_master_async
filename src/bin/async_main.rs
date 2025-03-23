//! SPI slave loopback test using DMA
//!
//! The following wiring is assumed for the (bitbang) slave:
//!
//! - SCLK => GPIO10
//! - MISO => GPIO11
//! - MOSI => GPIO12
//! - CS   => GPIO13
//!
//! 13 -> 21
//! 12 -> 22
//! 11 -> 35
//! 10 -> 27
//!
//! The following wiring is assumed for the (bitbang) master:
//! - SCLK => GPIO27
//! - MISO => GPIO35
//! - MOSI => GPIO22
//! - CS   => GPIO21
//!
//! Depending on your target and the board you are using you have to change the
//! pins.
//!
//! This example transfers data via SPI.
//!
//! Connect corresponding master and slave pins to see the outgoing data is read
//! as incoming data. The master-side pins are chosen to make these connections
//! easy for the barebones chip; all are immediate neighbors of the slave-side
//! pins except SCLK. SCLK is between MOSI and VDD3P3_RTC on the barebones chip,
//! so no immediate neighbor is available.

//% CHIPS: esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: esp-hal/unstable

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use embassy_time::{Duration, Timer};
use embedded_hal::digital::OutputPin;
use esp_backtrace as _;
use esp_hal::{
    gpio::{Event, Io},
    handler,
    peripheral::Peripheral,
    ram,
    spi::master::Spi,
    timer::timg::TimerGroup,
};

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    dma_buffers,
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    main, spi,
};
use esp_println::println;

static mut MASTER_RECEIVE: [u8; 32000] = [0; 32000];
static mut MASTER_SEND: [u8; 32000] = [0; 32000];

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    println!("SPI slave loopback test using DMA");
    println!("This example transfers data via SPI.");

    let peripherals = esp_hal::init(esp_hal::Config::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);
    /*
        let mut master_sclk = Output::new(peripherals.GPIO27, Level::Low, OutputConfig::default());
        let master_miso = Input::new(
            peripherals.GPIO35,
            InputConfig::default().with_pull(Pull::None),
        );
        let mut master_mosi = Output::new(peripherals.GPIO22, Level::Low, OutputConfig::default());

        */
    let mut master_cs = Output::new(peripherals.GPIO21, Level::High, OutputConfig::default());
    
    let mut master_send = unsafe { &mut MASTER_SEND };
    let mut master_receive = unsafe { &mut MASTER_RECEIVE };

    let mut i = 0;

    // Initialize master_send with a simple pattern
    master_send.fill(0xAA); // Fill with pattern 10101010

    // Set up the CS interrupt
    spawner.spawn(background_task()).unwrap();

    let mut master_spi = Spi::new(
        peripherals.SPI3,
        esp_hal::spi::master::Config::default()
            .with_frequency(esp_hal::time::Rate::from_khz(100))
            .with_mode(esp_hal::spi::Mode::_0),
    )
    .unwrap()
    .with_sck(peripherals.GPIO27)
    .with_mosi(peripherals.GPIO22)
    .with_miso(peripherals.GPIO35)
    //.with_cs(master_cs)
    .into_async();

    loop {
        // Update a few key positions to track iterations
        master_send[0] = i; // First byte shows iteration count
        master_send[1] = 0xBB; // Fixed marker
        master_send[2] = 0xCC; // Fixed marker
        master_send[master_send.len() - 1] = i; // Last byte also shows iteration count

        i = i.wrapping_add(1); // Increment counter, wrapping at 255

        //write_data_spi_async(&mut master_spi, master_send, &mut master_cs).await.unwrap();
        // Write data to SPI
        master_cs.set_low();
        master_spi.write(master_send).unwrap();
        master_cs.set_high();
        println!(
            "sent stuff {:x?} .. {:x?}",
            &master_send[..10],
            &master_send[master_send.len() - 10..],
        );
        Timer::after(Duration::from_millis(5000)).await;
    }
}

async fn write_data_spi_async(
    spi: &mut Spi<'_, esp_hal::Async>,
    data: &[u8], cs: &mut Output<'_>,
) -> Result<(), spi::Error> {
    // Write data to SPI
    println!("cs is low now");
    cs.set_low(); // Set CS low to start transmission
    embedded_hal_async::spi::SpiBus::write(spi, data).await?;
    cs.set_high(); // Set CS high to end transmission
    Ok(()) // Return Ok to satisfy the Result return type
}

// This task runs in the background and can be used to perform other operations
#[embassy_executor::task]
async fn background_task() {
    loop {
        // Wait for a short duration between checks
        Timer::after(Duration::from_millis(1000)).await;

        // Print a message to show the task is running
        println!("Just some background task...");
    }
}
