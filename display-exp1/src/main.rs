//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use adafruit_macropad::hal::gpio::{FunctionSpi, Input, PullUp};
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
use rotary_encoder_hal::{Direction, Rotary};
use rp_pico::hal::gpio::PushPullOutput;
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

    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);

    let _spi_sclk = pins.sclk.into_mode::<FunctionSpi>();
    let _spi_sclk = pins.mosi.into_mode::<FunctionSpi>();
    // let _ = pins.miso.into_mode::<PushPullOutput>();

    let spi1 = Spi::<_, _, 8>::new(pac.SPI1).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16_000_000u32.Hz(),
        &MODE_0,
    );

    //

    let mut oled_reset = pins.oled_reset.into_mode::<PushPullOutput>();

    let mut disp: GraphicsMode<_> = Builder::new()
        .connect_spi(
            spi1,
            pins.oled_dc.into_mode::<PushPullOutput>(),
            pins.oled_cs.into_mode::<PushPullOutput>(),
        )
        .into();

    disp.reset(&mut oled_reset, &mut delay).unwrap();

    disp.init().unwrap();
    disp.flush().unwrap();
    disp.clear();

    let mut led_pin = pins.led.into_push_pull_output();

    let mut neopixels = macropad_neopixels!(pins, clocks, timer, pac);

    //

    let mut rotary = Rotary::new(
        pins.encoder_rota.into_mode::<Input<PullUp>>(),
        pins.encoder_rotb.into_mode::<Input<PullUp>>(),
    );

    //
    //

    let _ = disp.draw_iter((0..10).flat_map(move |y| {
        (70..90).map(move |x| {
            embedded_graphics_core::Pixel(
                embedded_graphics_core::prelude::Point::new(x, y),
                if 0 == 1 & (x + y) {
                    embedded_graphics_core::pixelcolor::BinaryColor::On
                } else {
                    embedded_graphics_core::pixelcolor::BinaryColor::Off
                },
            )
        })
    }));

    let _ = disp.flush();

    let _ = graphics_exp::try_to_draw(&mut disp);

    let mut bright = 200;

    let mut ticks: u16 = 0;
    loop {
        let black = [(0, 0, 0); 3];
        let rgb = [(bright, 0, 0), (0, bright, 0), (0, 0, bright)];
        let cmy = [
            (0, bright, bright),
            (bright, 0, bright),
            (bright, bright, 0),
        ];

        // const DECODE: [i8; 4] = [0, 1, 3, 2];
        // let phase = DECODE[rotary.state as usize];
        let phase = bright as i8;

        if ticks & 0x200 == 0 {
            led_pin.set_high().unwrap();
            let _ = neopixels.write(wacky_lights(&black, &cmy, phase));
        } else {
            led_pin.set_low().unwrap();
            let _ = neopixels.write(wacky_lights(&black, &rgb, phase));
        }

        match rotary.update() {
            Ok(Direction::CounterClockwise) => {
                bright = (bright as u16 + 1).min(255) as u8;
            }
            Ok(Direction::Clockwise) => {
                bright = (bright as i16 - 1).max(0) as u8;
            }
            Ok(Direction::None) => {}
            Err(_) => {
                // don't care
            }
        }

        ticks = ticks.wrapping_add(1);

        delay.delay_ms(1);
    }
}

fn wacky_lights<'a>(
    black: &'a [(u8, u8, u8); 3],
    rgb: &'a [(u8, u8, u8); 3],
    phase: i8,
) -> impl Iterator<Item = (u8, u8, u8)> + 'a {
    (0..4)
        .flat_map(move |row| {
            if (row as i16 - phase as i16) % 4 == 0 {
                rgb.iter()
            } else {
                black.iter()
            }
        })
        .copied()
}
