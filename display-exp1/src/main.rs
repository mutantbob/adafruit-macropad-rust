//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use adafruit_macropad::hal::pio::PIOExt;
use adafruit_macropad::hal::{Spi, Timer};
use adafruit_macropad::Pins;
use adafruit_macropad::XOSC_CRYSTAL_FREQ;
use bsp::entry;
use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};
use defmt::*;
use defmt_rtt as _;
use embedded_graphics_core::draw_target::DrawTarget as DT_;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::spi::MODE_0;
use embedded_time::fixed_point::FixedPoint;
use embedded_time::rate::Extensions;
use panic_probe as _;
use rp_pico::hal::gpio::Readable;
use sh1106::interface::{DisplayInterface, SpiInterface};
use sh1106::mode::GraphicsMode;
use sh1106::Builder;
use smart_leds_trait::SmartLedsWrite;
use ws2812_pio::Ws2812;

mod graphics_exp;

macro_rules! macropad_neopixels {
    ($pins:expr, $pio:expr, $sm0: expr, $clocks:expr, $timer:expr) => {
        Ws2812::new(
            $pins.neopixel.into_mode(),
            &mut $pio,
            $sm0,
            $clocks.peripheral_clock.freq(),
            $timer.count_down(),
        )
    };
    ($pins: expr, $clocks:expr, $timer:expr, $pac:expr) => {{
        let (mut pio, sm0, _, _, _) = $pac.PIO0.split(&mut $pac.RESETS);
        macropad_neopixels!($pins, pio, sm0, $clocks, $timer)
    }};
}

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

    // let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);

    let _ = pins
        .mosi
        .into_mode::<adafruit_macropad::hal::gpio::FunctionSpi>();
    let _ = pins
        .miso
        .into_mode::<adafruit_macropad::hal::gpio::FunctionSpi>();

    let spi1 = Spi::<_, _, 8>::new(pac.SPI1).init(
        &mut pac.RESETS,
        125_000_000u32.Hz(),
        1_000_000u32.Hz(),
        &MODE_0,
    );

    //

    let mut disp: GraphicsMode<
        SpiInterface<
            Spi<_, _, 8_u8>,
            adafruit_macropad::hal::gpio::Pin<_, adafruit_macropad::hal::gpio::Output<Readable>>,
            adafruit_macropad::hal::gpio::Pin<_, adafruit_macropad::hal::gpio::Output<Readable>>,
        >,
    > = Builder::new()
        .connect_spi(spi1, pins.oled_dc.into_mode(), pins.oled_cs.into_mode())
        .into();

    disp.init().unwrap();
    disp.flush().unwrap();
    disp.clear();

    check3(&mut disp);
    // check2(&mut disp);

    let mut led_pin = pins.led.into_push_pull_output();

    let mut neopixels = macropad_neopixels!(pins, clocks, timer, pac);

    //
    //

    let _ = disp.draw_iter(
        (0..5)
            .map(move |y| {
                (0..10).map(move |x| {
                    embedded_graphics_core::Pixel(
                        embedded_graphics_core::prelude::Point::new(x, y),
                        if 0 == 1 & (x + y) {
                            embedded_graphics_core::pixelcolor::BinaryColor::On
                        } else {
                            embedded_graphics_core::pixelcolor::BinaryColor::Off
                        },
                    )
                })
            })
            .flatten(),
    );

    let _ = disp.flush();

    let _ = graphics_exp::try_to_draw(&mut disp);

    let mut bright = 255;
    loop {
        let rgb = [(bright, 0, 0), (0, bright, 0), (0, 0, bright)];
        let cmy = [
            (0, bright, bright),
            (bright, 0, bright),
            (bright, bright, 0),
        ];
        led_pin.set_high().unwrap();
        let _ = neopixels.write(cmy.iter().copied());
        delay.delay_ms(1500);
        led_pin.set_low().unwrap();
        let _ = neopixels.write(rgb.iter().copied());
        delay.delay_ms(1500);
    }
}

/*pub fn check1<DI>(_x: &SpiInterface<DI, DC, CS>)
{

}*/

// pub fn check2<D: embedded_graphics::DrawTarget>(_x: &mut D) {}
pub fn check3<DI: DisplayInterface>(_x: &mut GraphicsMode<DI>) {}

// End of file
