//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use core::str::from_utf8_unchecked;
// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use adafruit_macropad::hal::gpio::{Input, PullUp};
use adafruit_macropad::hal::pio::PIOExt;
use adafruit_macropad::hal::Timer;
use adafruit_macropad_macros::{macropad_clocks, macropad_neopixels, macropad_oled, macropad_pins};
use bsp::entry;
use bsp::hal::{clocks::Clock, pac, watchdog::Watchdog};
use defmt::*;
use defmt_rtt as _;
use embedded_graphics::fonts::{Font6x8, Text};
use embedded_graphics::style::TextStyle;
use embedded_graphics_core::draw_target::DrawTarget as DT_;
use embedded_graphics_core::geometry::{Point, Size};
use embedded_graphics_core::pixelcolor::BinaryColor;
use embedded_graphics_core::primitives::Rectangle;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::fixed_point::FixedPoint;
use macropad_helpers::DTWrapper;
use panic_probe as _;
use rotary_encoder_hal::{Direction, Rotary};
use smart_leds_trait::SmartLedsWrite;
use ufmt::{uWrite, uwrite};

mod graphics_exp;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let clocks = macropad_clocks!(pac, watchdog);

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let pins = macropad_pins!(pac);

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);

    //

    let mut disp = macropad_oled!(pins, clocks, delay, pac);

    let mut led_pin = pins.led.into_push_pull_output();

    let mut neopixels = macropad_neopixels!(pins, clocks, timer, pac);

    //

    let mut rotary = Rotary::new(
        pins.encoder_rota.into_mode::<Input<PullUp>>(),
        pins.encoder_rotb.into_mode::<Input<PullUp>>(),
    );

    //
    //

    disp.clear();

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
    let mut old_brightness = ChangeDetector::new(0);

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
            Ok(Direction::Clockwise) => {
                bright = (bright as u16 + 1).min(255) as u8;
            }
            Ok(Direction::CounterClockwise) => {
                bright = (bright as i16 - 1).max(0) as u8;
            }
            Ok(Direction::None) => {}
            Err(_) => {
                // don't care
            }
        }

        if old_brightness.changed(bright) {
            use embedded_graphics::drawable::Drawable;
            let p1 = Point::new(20, 40);
            let _ = disp.fill_solid(
                &Rectangle::new(p1.clone(), Size::new(100, 60)),
                BinaryColor::On,
            );

            let mut fmt_buffer = UfmtWrapper::<80>::new();

            // fmt_buffer.write_str("bright");
            uwrite!(&mut fmt_buffer, "brightness={}", bright).unwrap();

            let _ = Text::new(
                fmt_buffer.as_str(),
                // "brightness",
                embedded_graphics::geometry::Point::new(p1.x + 1, p1.y + 1),
            )
            .into_styled(TextStyle::new(
                Font6x8,
                embedded_graphics::pixelcolor::BinaryColor::Off,
            ))
            .draw(&mut DTWrapper { inner: &mut disp });
            let _ = disp.flush();
        }

        ticks = ticks.wrapping_add(1);

        delay.delay_ms(1);
    }
}

struct ChangeDetector<T> {
    old: T,
}

impl<T: PartialEq> ChangeDetector<T> {
    pub fn new(first: T) -> Self {
        ChangeDetector { old: first }
    }

    pub fn changed(&mut self, new_val: T) -> bool {
        if new_val == self.old {
            false
        } else {
            self.old = new_val;
            true
        }
    }
}

struct UfmtWrapper<const N: usize> {
    cursor: usize,
    buffer: [u8; N],
}

impl<const N: usize> UfmtWrapper<N> {
    pub fn new() -> Self {
        UfmtWrapper {
            cursor: 0,
            buffer: [0; N],
        }
    }

    pub fn as_str<'a>(&'a self) -> &'a str {
        unsafe { from_utf8_unchecked(&self.buffer[..self.cursor]) }
    }
}

impl<const N: usize> uWrite for UfmtWrapper<N> {
    type Error = ();

    fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
        let bytes = s.as_bytes();
        self.buffer[self.cursor..(self.cursor + bytes.len())].copy_from_slice(bytes);
        self.cursor += bytes.len();
        Ok(())
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
