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

use crate::pac::PIO0;
use adafruit_macropad::hal::gpio::{Function, Pin, PinId, PushPullOutput, ValidPinMode};
use adafruit_macropad::hal::pio::{PIOExt, SM0};
use adafruit_macropad::hal::timer::CountDown;
use adafruit_macropad::hal::usb::UsbBus;
use adafruit_macropad::hal::Timer;
use adafruit_macropad_macros::{
    macropad_clocks, macropad_keypad, macropad_neopixels, macropad_oled, macropad_pins,
    macropad_rotary_encoder, KeysTwelve,
};
use bsp::entry;
use bsp::hal::{clocks::Clock, pac, watchdog::Watchdog};
use defmt::*;
use defmt_rtt as _;
use embedded_graphics::drawable::Drawable;
use embedded_graphics::fonts::{Font6x8, Text};
use embedded_graphics::style::TextStyle;
use embedded_graphics_core::draw_target::DrawTarget as DT_;
use embedded_graphics_core::geometry::{Point, Size};
use embedded_graphics_core::pixelcolor::BinaryColor;
use embedded_graphics_core::primitives::Rectangle;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_time::fixed_point::FixedPoint;
use macropad_helpers::DTWrapper;
use panic_probe as _;
use rotary_encoder_hal::{Direction, Rotary};
use smart_leds_trait::SmartLedsWrite;
use ufmt::{uWrite, uwrite};
use usb_device::bus::UsbBusAllocator;
use usb_device::device::{UsbDevice, UsbDeviceBuilder, UsbDeviceState, UsbVidPid};
use usbd_hid::descriptor::{KeyboardReport, SerializedDescriptor};
use usbd_hid::hid_class::HIDClass;
use ws2812_pio::Ws2812;

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

    let mut rotary = macropad_rotary_encoder!(pins);

    let keys = macropad_keypad!(pins);

    let usb_bus = UsbBusAllocator::new(adafruit_macropad::hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    let mut keyboard_hid = HIDClass::new(&usb_bus, KeyboardReport::desc(), 10);
    let mut usb_device = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x239a, 0xbeef))
        .product("macropad-usbd")
        .manufacturer("Bob's mad science")
        .build();

    loop {
        //support::poll_logger();
        if !usb_device.poll(&mut [&mut keyboard_hid]) {
            continue;
        }
        let state = usb_device.state();
        if state == UsbDeviceState::Configured {
            break;
        }
    }

    // usb_device.bus().configure();

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

    let mut application = ApplicationLoop::new();
    loop {
        application.one_pass(
            &mut disp,
            &mut led_pin,
            &mut neopixels,
            &mut rotary,
            &keys,
            &mut keyboard_hid,
            &mut usb_device,
        );
        let _ = disp.flush();
        delay.delay_ms(1);
    }
}

struct ApplicationLoop {
    bright: u8,
    old_brightness: ChangeDetector<u8>,
    ticks: u32,
    key_state: [bool; 12],
    old_key_state: ChangeDetector<[bool; 12]>,
}

impl ApplicationLoop {
    pub fn new() -> Self {
        ApplicationLoop {
            bright: 200,
            old_brightness: ChangeDetector::new(0),
            ticks: 0,
            key_state: [false; 12],
            old_key_state: ChangeDetector::new([true; 12]),
        }
    }

    pub fn one_pass<D, LED, NEOPIN, RA, RB, E>(
        &mut self,
        disp: &mut D,
        led_pin: &mut Pin<LED, PushPullOutput>,
        neopixels: &mut Ws2812<PIO0, SM0, CountDown, NEOPIN>,
        rotary: &mut Rotary<RA, RB>,
        keys: &KeysTwelve,
        hid: &mut HIDClass<UsbBus>,
        usb_device: &mut UsbDevice<UsbBus>,
    ) where
        D: embedded_graphics_core::draw_target::DrawTarget<
            Color = embedded_graphics_core::pixelcolor::BinaryColor,
            Error = E,
        >,
        LED: PinId,
        RA: InputPin,
        RB: InputPin,
        NEOPIN: PinId,
        Function<PIO0>: ValidPinMode<NEOPIN>,
    {
        let bright = self.bright;
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

        if self.ticks & 0x200 == 0 {
            led_pin.set_high().unwrap();
            let _ = neopixels.write(wacky_lights(&black, &cmy, phase));
        } else {
            led_pin.set_low().unwrap();
            let _ = neopixels.write(wacky_lights(&black, &rgb, phase));
        }

        match rotary.update() {
            Ok(Direction::Clockwise) => {
                self.bright = (bright as u16 + 1).min(255) as u8;
            }
            Ok(Direction::CounterClockwise) => {
                self.bright = (bright as i16 - 1).max(0) as u8;
            }
            Ok(Direction::None) => {}
            Err(_) => {
                // don't care
            }
        }

        self.key_state[0] = keys.key1.is_low().unwrap();

        usb_device.poll(&mut [hid]);
        if self.key_state[0] {
            let _ = hid.push_input(&simple_kr1(0, b'w' - b'a' + 4));
        } else {
            let _ = hid.push_input(&simple_kr1(0, 0));
        }

        self.update_display(disp);

        self.ticks = self.ticks.wrapping_add(1);
    }

    fn update_display<D, E>(&mut self, disp: &mut D)
    where
        D: embedded_graphics_core::draw_target::DrawTarget<
            Color = embedded_graphics_core::pixelcolor::BinaryColor,
            Error = E,
        >,
    {
        if self.old_brightness.changed(self.bright) {
            let p1 = Point::new(20, 40);
            let _ = disp.fill_solid(&Rectangle::new(p1, Size::new(100, 60)), BinaryColor::On);

            let mut fmt_buffer = UfmtWrapper::<80>::new();

            // fmt_buffer.write_str("bright");
            uwrite!(&mut fmt_buffer, "brightness={}", self.bright).unwrap();

            easy_text_at(
                fmt_buffer.as_str(),
                // "brightness",
                p1.x + 1,
                p1.y + 1,
                disp,
                embedded_graphics::pixelcolor::BinaryColor::Off,
            );
        }

        if self.old_key_state.changed(self.key_state) {
            let p1 = Point::new(20, 1);
            let _ = disp.fill_solid(&Rectangle::new(p1, Size::new(100, 10)), BinaryColor::Off);
            easy_text_at(
                if self.key_state[0] { "0down" } else { "0up" },
                p1.x + 1,
                p1.y + 1,
                disp,
                embedded_graphics::pixelcolor::BinaryColor::On,
            );
        }
    }
}

fn simple_kr1(modifier: u8, key_code_1: u8) -> KeyboardReport {
    KeyboardReport {
        modifier: modifier,
        reserved: 0,
        leds: 0,
        keycodes: [key_code_1, 0, 0, 0, 0, 0],
    }
}

pub fn easy_text_at<E, D>(
    msg: &str,
    x: i32,
    y: i32,
    disp: &mut D,
    color: embedded_graphics::pixelcolor::BinaryColor,
) where
    D: embedded_graphics_core::draw_target::DrawTarget<
        Color = embedded_graphics_core::pixelcolor::BinaryColor,
        Error = E,
    >,
{
    let _ = Text::new(msg, embedded_graphics::geometry::Point::new(x, y))
        .into_styled(TextStyle::new(Font6x8, color))
        .draw(&mut DTWrapper { inner: disp });
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

    pub fn as_str(&self) -> &str {
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
