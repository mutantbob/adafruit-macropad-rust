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

use crate::core1::{InterCoreMessage, JogAxis, LastJog};
use crate::keybindings::{
    key_x_minus, key_x_plus, key_y_minus, key_y_plus, key_z_minus, key_z_plus,
};
use adafruit_macropad::hal::gpio::{Function, Pin, PinId, PushPullOutput, ValidPinMode};
use adafruit_macropad::hal::multicore::{Multicore, Stack};
use adafruit_macropad::hal::pio::SM0;
use adafruit_macropad::hal::sio::SioFifo;
use adafruit_macropad::hal::timer::CountDown;
use adafruit_macropad::hal::usb::UsbBus;
use adafruit_macropad::hal::Timer;
use adafruit_macropad::{
    macropad_clocks, macropad_keypad, macropad_neopixels, macropad_oled, macropad_pins,
    macropad_rotary_encoder, Sio,
};
use bsp::entry;
use bsp::hal::pac::PIO0;
use bsp::hal::{clocks::Clock, pac, watchdog::Watchdog};
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::fixed_point::FixedPoint;
use painter::DisplayPainter;
use panic_probe as _;
use smart_leds_trait::SmartLedsWrite;
use ufmt::{uWrite, uwrite};
use usb_device::bus::UsbBusAllocator;
use usb_device::device::{UsbDevice, UsbDeviceBuilder, UsbDeviceState, UsbVidPid};
use usbd_hid::descriptor::{KeyboardReport, MouseReport, SerializedDescriptor};
use usbd_hid::hid_class::HIDClass;
use ws2812_pio::Ws2812;

mod core1;
mod keybindings;
mod painter;

static mut CORE1_STACK: Stack<4096> = Stack::new();

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let clocks = macropad_clocks!(pac, watchdog);

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());
    let mut sio = Sio::new(pac.SIO);
    let pins = macropad_pins!(pac, sio);

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);

    //

    let mut disp = macropad_oled!(pins, clocks, delay, pac);

    let mut led_pin = pins.led.into_push_pull_output();

    let mut neopixels = macropad_neopixels!(pins, clocks, timer, pac);

    //

    let rotary = macropad_rotary_encoder!(pins);

    let keys = macropad_keypad!(pins);

    let usb_bus = UsbBusAllocator::new(adafruit_macropad::hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    let mut keyboard_hid = HIDClass::new(&usb_bus, KeyboardReport::desc(), 10);
    let mut mouse_hid = HIDClass::new(&usb_bus, MouseReport::desc(), 10);
    let mut usb_device = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x239a, 0xbeef))
        .product("macropad-usbd")
        .manufacturer("Bob's mad science")
        .build();

    loop {
        //support::poll_logger();
        if !usb_device.poll(&mut [&mut keyboard_hid, &mut mouse_hid]) {
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

    //let sys_freq = clocks.system_clock.freq().integer();
    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    let _test = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
        core1::core1_task(rotary, keys)
    });

    let mut application = ApplicationLoop::new(
        disp,
        &mut led_pin,
        &mut neopixels,
        sio.fifo,
        &mut usb_device,
        &mut keyboard_hid,
    );

    loop {
        let usec = timer.get_counter();
        application.one_pass((usec / 1000) as u32);
        let _ = application.display_painter.disp.flush();
        //delay.delay_ms(1);
    }
}

//

struct UsbGenerator<'a> {
    usb_device: &'a mut UsbDevice<'a, UsbBus>,
    keyboard_hid: &'a mut HIDClass<'a, UsbBus>,
    debug_msg: UfmtWrapper<200>,
}

impl<'a> UsbGenerator<'a> {
    pub fn new(
        usb_device: &'a mut UsbDevice<'a, UsbBus>,
        keyboard_hid: &'a mut HIDClass<'a, UsbBus>,
    ) -> Self {
        UsbGenerator {
            usb_device,
            keyboard_hid,
            debug_msg: UfmtWrapper::new(),
        }
    }

    pub fn generate_usb_events(
        &mut self,
        jog_axis: JogAxis,
        jog_action: LastJog,
        key_pressed: Option<u8>,
    ) {
        self.debug_msg.reset();

        let kr = Self::simple_keyboard_action(key_pressed);

        let _ = uwrite!(&mut self.debug_msg, "k {:?}", key_pressed);

        self.send_keyboard_report(&kr);
    }

    pub fn send_keyboard_report(&mut self, kr: &KeyboardReport) -> bool {
        self.usb_device.poll(&mut [self.keyboard_hid]);

        self.keyboard_hid.push_input(kr).is_ok()
    }

    pub fn simple_keyboard_action(key_pressed: Option<u8>) -> KeyboardReport {
        let kr: KeyboardReport = match key_pressed {
            Some(1) => key_y_plus(),
            Some(3) => key_x_minus(),
            Some(5) => key_x_plus(),
            Some(7) => key_y_minus(),
            Some(2) => key_z_plus(),
            Some(8) => key_z_minus(),
            _ => keybindings::simple_kr1(0, 0),
        };
        kr
    }
}

//

struct ApplicationLoop<'a, D, LED, NEOPIN, E>
where
    D: embedded_graphics_core::draw_target::DrawTarget<
        Color = embedded_graphics_core::pixelcolor::BinaryColor,
        Error = E,
    >,
    LED: PinId,
    NEOPIN: PinId,
    Function<PIO0>: ValidPinMode<NEOPIN>,
{
    display_painter: DisplayPainter<D, E>,
    led_pin: &'a mut Pin<LED, PushPullOutput>,
    neopixels: &'a mut Ws2812<PIO0, SM0, CountDown<'a>, NEOPIN>,

    last_key: Option<u8>,
    jog_axis: JogAxis,
    jog_scale: u8,
    jog_scale_down: bool,

    fifo: SioFifo,
    generator: UsbGenerator<'a>,
}

impl<'a, D, LED, NEOPIN, E> ApplicationLoop<'a, D, LED, NEOPIN, E>
where
    D: embedded_graphics_core::draw_target::DrawTarget<
        Color = embedded_graphics_core::pixelcolor::BinaryColor,
        Error = E,
    >,
    LED: PinId,

    NEOPIN: PinId,
    Function<PIO0>: ValidPinMode<NEOPIN>,
{
    pub fn new(
        disp: D,
        led_pin: &'a mut Pin<LED, PushPullOutput>,
        neopixels: &'a mut Ws2812<PIO0, SM0, CountDown<'a>, NEOPIN>,
        fifo: SioFifo,
        usb_device: &'a mut UsbDevice<'a, UsbBus>,
        keyboard_hid: &'a mut HIDClass<'a, UsbBus>,
    ) -> Self {
        ApplicationLoop {
            display_painter: DisplayPainter::new(disp),
            led_pin,
            neopixels,
            last_key: None,
            jog_axis: JogAxis::X,
            jog_scale: 0,
            jog_scale_down: false,
            fifo,
            generator: UsbGenerator::new(usb_device, keyboard_hid),
        }
    }

    pub fn one_pass(&mut self, epoch_millis: u32) {
        if epoch_millis & 0x100 == 0 {
            self.led_pin.set_high().unwrap();
        } else {
            self.led_pin.set_low().unwrap();
        }

        let (jog_action, _) = match poll_core1(&mut self.fifo) {
            Some(msg) => {
                self.last_key = msg.last_key;
                self.jog_axis = msg.jog_axis;
                (msg.jog_action, msg.jog_scale)
            }
            None => (LastJog::None, self.jog_scale),
        };

        let jog_scale = if self.last_key == Some(4) {
            if !self.jog_scale_down {
                (1 + self.jog_scale) % 4
            } else {
                self.jog_scale
            }
        } else {
            self.jog_scale_down = false;
            self.jog_scale
        };

        let _ = self
            .neopixels
            .write(lights_for(self.jog_axis, self.last_key));

        if jog_scale != self.jog_scale {
            let kr = keybindings::key_jog_scale(jog_scale);
            if self.generator.send_keyboard_report(&kr) {
                self.jog_scale = jog_scale;
                self.jog_scale_down = true;
            }
        } else {
            self.generator
                .generate_usb_events(self.jog_axis, jog_action, self.last_key);
        }

        if false {
            let _ = self.display_painter.update_display(
                0,
                [false; 12],
                None,
                self.generator.debug_msg.as_str(),
            );
            self.generator.debug_msg.reset();
        } else {
            let _ = self.display_painter.idle_display();
        }
    }
}

fn poll_core1(fifo: &mut SioFifo) -> Option<InterCoreMessage> {
    if !fifo.is_write_ready() {
        return None;
    }

    fifo.write(0);

    InterCoreMessage::receive(fifo).ok()
}

fn lights_for(jog_axis: JogAxis, last_key: Option<u8>) -> impl Iterator<Item = (u8, u8, u8)> {
    const X_COLOR: (u8, u8, u8) = (255, 0, 255);
    const Y_COLOR: (u8, u8, u8) = (0, 255, 255);
    const Z_COLOR: (u8, u8, u8) = (0, 0, 255);

    let mut rgbs = [
        (0, 0, 0),
        Y_COLOR,
        Z_COLOR,
        X_COLOR,
        (255, 255, 0),
        X_COLOR,
        (0, 0, 0),
        Y_COLOR,
        Z_COLOR,
        X_COLOR,
        Y_COLOR,
        Z_COLOR,
    ];

    if let Some(idx) = last_key {
        rgbs[idx as usize] = (255, 255, 255);
    }

    {
        rgbs[(jog_axis.as_int() + 9) as usize] = (0, 255, 255);
    }

    let dim = 32;
    rgbs[9] = if jog_axis == JogAxis::X {
        X_COLOR
    } else {
        (dim, 0, dim)
    };
    rgbs[10] = if jog_axis == JogAxis::Y {
        Y_COLOR
    } else {
        (0, dim, dim)
    };
    rgbs[11] = if jog_axis == JogAxis::Z {
        Z_COLOR
    } else {
        (0, 0, dim)
    };

    rgbs.into_iter()
}

struct ChangeDetector<T> {
    pub old: T,
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

//
//

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
    pub fn reset(&mut self) {
        self.cursor = 0;
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

//
