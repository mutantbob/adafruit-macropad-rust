//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use adafruit_macropad::hal::pio::PIOExt;
use adafruit_macropad::hal::Timer;
use adafruit_macropad::Pins;
use adafruit_macropad::XOSC_CRYSTAL_FREQ;
use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::fixed_point::FixedPoint;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};
//use embedded_hal::prelude::_embedded_hal_blocking_spi_Write;
use ws2812_pio::Ws2812;
use smart_leds_trait::SmartLedsWrite;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let (mut pio, sm0, _,_,_) = pac.PIO0.split(&mut pac.RESETS);

    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);

    let mut led_pin = pins.led.into_push_pull_output();

    let neopixel = pins.neopixel;

    let mut neopixels = Ws2812::new(
        neopixel.into_mode(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    let mut bright = 255;
    loop {
        let rgb = [(bright, 0, 0), (0, bright, 0), (0, 0, bright)];
        let cmy = [(0, bright, bright), (bright, 0, bright), (bright, bright, 0)];
        led_pin.set_high().unwrap();
        let _ = neopixels.write(cmy.iter().copied());
        delay.delay_ms(1500);
        led_pin.set_low().unwrap();
        let _ = neopixels.write(rgb.iter().copied());
        delay.delay_ms(1500);
    }
}

// End of file
