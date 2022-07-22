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

use adafruit_macropad::hal::gpio::{Function, Pin, PinId, PushPullOutput, ValidPinMode};
use adafruit_macropad::hal::pio::{PIOExt, SM0};
use adafruit_macropad::hal::timer::CountDown;
use adafruit_macropad::hal::usb::UsbBus;
use adafruit_macropad::hal::Timer;
use adafruit_macropad::{
    macropad_clocks, macropad_keypad, macropad_neopixels, macropad_oled, macropad_pins,
    macropad_rotary_encoder, KeysTwelve,
};
use bsp::entry;
use bsp::hal::pac::PIO0;
use bsp::hal::{clocks::Clock, pac, watchdog::Watchdog};
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_time::fixed_point::FixedPoint;
use mission_modes::{simple_kr1, IcarusJog, KeyboardOrMouse, MissionMode, MouseHold};
use painter::DisplayPainter;
use panic_probe as _;
use rotary_encoder_hal::{Direction, Rotary};
use smart_leds_trait::SmartLedsWrite;
use ufmt::{uWrite, uwrite};
use usb_device::bus::UsbBusAllocator;
use usb_device::device::{UsbDevice, UsbDeviceBuilder, UsbDeviceState, UsbVidPid};
use usb_device::UsbError;
use usbd_hid::descriptor::{KeyboardReport, MouseReport, SerializedDescriptor};
use usbd_hid::hid_class::HIDClass;
use ws2812_pio::Ws2812;

mod mission_modes;
mod painter;

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

    let mut application = ApplicationLoop::new(
        disp,
        &mut led_pin,
        &mut neopixels,
        &mut rotary,
        &keys,
        &mut usb_device,
        &mut keyboard_hid,
        &mut mouse_hid,
    );
    loop {
        let usec = timer.get_counter();
        application.one_pass((usec / 1000) as u32);
        let _ = application.display_painter.disp.flush();
        //delay.delay_ms(1);
    }
}

struct MissionModes {
    icarus_jog_100: IcarusJog,
    icarus_jog_60: IcarusJog,
    icarus_jog_70: IcarusJog,
    icarus_jog_0: IcarusJog,
    mouse1: MouseHold,
}

impl MissionModes {
    pub(crate) fn mission_for(&mut self, idx: u8) -> Option<&mut dyn MissionMode<KeyboardOrMouse>> {
        match idx {
            0 => Some(&mut self.icarus_jog_100),
            3 => Some(&mut self.icarus_jog_70),
            6 => Some(&mut self.icarus_jog_60),
            9 => Some(&mut self.icarus_jog_0),

            1 => Some(&mut self.mouse1),

            _ => None,
        }
    }
}

impl MissionModes {
    pub fn new() -> Self {
        MissionModes {
            icarus_jog_100: IcarusJog::new(1.0),
            icarus_jog_60: IcarusJog::new(0.7),
            icarus_jog_70: IcarusJog::new(0.6),
            icarus_jog_0: IcarusJog::new(0.0),
            mouse1: MouseHold::new(),
        }
    }
}

//

#[derive(PartialEq)]
pub enum MissionUpdate {
    Change(Option<u8>),
    Preserve,
}

impl MissionUpdate {
    pub fn should_change(&self, other: Option<u8>) -> Option<Option<u8>> {
        match self {
            MissionUpdate::Change(future) => {
                if *future == other {
                    None
                } else {
                    Some(*future)
                }
            }
            MissionUpdate::Preserve => None,
        }
    }
}

impl ufmt::uDebug for MissionUpdate {
    fn fmt<W>(&self, ufmt: &mut ufmt::Formatter<'_, W>) -> Result<(), W::Error>
    where
        W: uWrite + ?Sized,
    {
        match self {
            MissionUpdate::Change(x) => {
                ufmt.write_str("Change(")?;
                x.fmt(ufmt)?;
                ufmt.write_str(")")
            }
            MissionUpdate::Preserve => ufmt.write_str("Preserve"),
        }
    }
}

//

struct UsbGenerator<'a> {
    modes: MissionModes,
    usb_device: &'a mut UsbDevice<'a, UsbBus>,
    keyboard_hid: &'a mut HIDClass<'a, UsbBus>,
    mouse_hid: &'a mut HIDClass<'a, UsbBus>,
    pushed_back: Option<KeyboardOrMouse>,
    active_mission: Option<u8>,
    new_mission: MissionUpdate,
    debug_msg: UfmtWrapper<200>,
}

impl<'a> UsbGenerator<'a> {
    pub fn new(
        usb_device: &'a mut UsbDevice<'a, UsbBus>,
        keyboard_hid: &'a mut HIDClass<'a, UsbBus>,
        mouse_hid: &'a mut HIDClass<'a, UsbBus>,
    ) -> Self {
        UsbGenerator {
            modes: MissionModes::new(),
            usb_device,
            keyboard_hid,
            mouse_hid,
            pushed_back: None,
            active_mission: None,
            new_mission: MissionUpdate::Preserve,
            debug_msg: UfmtWrapper::new(),
        }
    }

    fn curr(&mut self) -> Option<&mut dyn MissionMode<KeyboardOrMouse>> {
        self.active_mission
            .and_then(|idx| self.modes.mission_for(idx))
    }

    pub fn response(&mut self, millis_elapsed: u32) -> Option<KeyboardOrMouse> {
        match self.new_mission.should_change(self.active_mission) {
            Some(new_mission) => {
                let delay_reboot = self.curr().and_then(|mission| mission.maybe_deactivate());
                match delay_reboot {
                    None => {
                        self.active_mission = new_mission;
                        self.new_mission = MissionUpdate::Preserve;
                        if let Some(curr) = self.curr() {
                            curr.reboot();
                            Some(curr.one_usb_pass(millis_elapsed))
                        } else {
                            None
                        }
                    }
                    Some(kr) => Some(kr),
                }
            }
            None => self
                .pushed_back
                .take()
                .or_else(|| self.curr().map(|curr| curr.one_usb_pass(millis_elapsed))),
        }
    }

    pub fn generate_usb_events(&mut self, fire_idx: Option<u8>, ticks: u32) {
        self.debug_msg.reset();

        if let Some(idx) = fire_idx {
            self.new_mission = if self.active_mission == Some(idx) {
                let _ = uwrite!(&mut self.debug_msg, "D ");
                MissionUpdate::Change(None)
            } else {
                MissionUpdate::Change(Some(idx))
            };
        }
        if self.new_mission == MissionUpdate::Preserve {
            self.debug_msg.reset();
        } else if false {
            let _ = uwrite!(&mut self.debug_msg, "{:?}", fire_idx);
            let _ = uwrite!(&mut self.debug_msg, "\n{:?}", self.new_mission);
            let _ = uwrite!(&mut self.debug_msg, " {:?}", self.active_mission);
        }

        self.usb_device
            .poll(&mut [self.keyboard_hid, self.mouse_hid]);

        let report = self.response(ticks);
        match report {
            Some(KeyboardOrMouse::Keyboard(report)) => {
                if let Err(UsbError::WouldBlock) = self.keyboard_hid.push_input(&report) {
                    self.pushed_back = Some(KeyboardOrMouse::Keyboard(report));
                }
            }
            Some(KeyboardOrMouse::Mouse(report)) => {
                if let Err(UsbError::WouldBlock) = self.mouse_hid.push_input(&report) {
                    self.pushed_back = Some(KeyboardOrMouse::Mouse(report));
                }
            }
            None => {
                let _ = self.keyboard_hid.push_input(&simple_kr1(0, 0));
                let _ = self.mouse_hid.push_input(&MouseReport {
                    buttons: 0,
                    x: 0,
                    y: 0,
                    wheel: 0,
                    pan: 0,
                });
            }
        }
    }
}

//

struct ApplicationLoop<'a, D, LED, NEOPIN, RA, RB, E>
where
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
    bright: u8,
    key_state: [bool; 12],
    old_key_state: ChangeDetector<[bool; 12]>,
    display_painter: DisplayPainter<D, E>,
    led_pin: &'a mut Pin<LED, PushPullOutput>,
    neopixels: &'a mut Ws2812<PIO0, SM0, CountDown<'a>, NEOPIN>,
    rotary: &'a mut Rotary<RA, RB>,
    keys: &'a KeysTwelve,
    generator: UsbGenerator<'a>,
}

impl<'a, D, LED, NEOPIN, RA, RB, E> ApplicationLoop<'a, D, LED, NEOPIN, RA, RB, E>
where
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
    pub fn new(
        disp: D,
        led_pin: &'a mut Pin<LED, PushPullOutput>,
        neopixels: &'a mut Ws2812<PIO0, SM0, CountDown<'a>, NEOPIN>,
        rotary: &'a mut Rotary<RA, RB>,
        keys: &'a KeysTwelve,
        usb_device: &'a mut UsbDevice<'a, UsbBus>,
        keyboard_hid: &'a mut HIDClass<'a, UsbBus>,
        mouse_hid: &'a mut HIDClass<'a, UsbBus>,
    ) -> Self {
        ApplicationLoop {
            bright: 200,
            key_state: [false; 12],
            old_key_state: ChangeDetector::new([true; 12]),
            display_painter: DisplayPainter::new(disp),
            led_pin,
            neopixels,
            rotary,
            keys,
            generator: UsbGenerator::new(usb_device, keyboard_hid, mouse_hid),
        }
    }

    pub fn one_pass(&mut self, epoch_millis: u32) {
        let phase = self.bright as i8;

        if epoch_millis & 0x100 == 0 {
            self.led_pin.set_high().unwrap();
        } else {
            self.led_pin.set_low().unwrap();
        }

        let _ = self.neopixels.write(lights_for(
            self.bright,
            epoch_millis,
            phase,
            self.generator.active_mission,
        ));

        match self.rotary.update() {
            Ok(Direction::Clockwise) => {
                self.bright = (self.bright as u16 + 1).min(255) as u8;
            }
            Ok(Direction::CounterClockwise) => {
                self.bright = (self.bright as i16 - 1).max(0) as u8;
            }
            Ok(Direction::None) => {}
            Err(_) => {
                // don't care
            }
        }

        {
            let keys_array = self.keys.array_0based();
            for (idx, state) in self.key_state.iter_mut().enumerate() {
                let pressed = keys_array[idx].is_low().unwrap();
                *state = pressed;
            }
        }

        let mut fire_array = [false; 12];
        for (idx, fire) in fire_array.iter_mut().enumerate() {
            *fire = self.key_state[idx] && !self.old_key_state.old[idx];
        }

        let fire_idx = fire_array
            .iter()
            .enumerate()
            .find(|(_, b)| **b)
            .map(|(idx, _)| idx as u8);

        self.generator.generate_usb_events(fire_idx, epoch_millis);

        if false {
            let _ = self.display_painter.update_display(
                self.bright,
                self.key_state,
                self.generator.active_mission,
                self.generator.debug_msg.as_str(),
            );
            self.generator.debug_msg.reset();
        } else {
            let _ = self.display_painter.idle_display();
        }

        self.old_key_state.changed(self.key_state);
    }
}

fn lights_for(
    bright: u8,
    ticks: u32,
    phase: i8,
    white_idx: Option<u8>,
) -> impl Iterator<Item = (u8, u8, u8)> {
    let black = [(0, 0, 0); 3];
    let rgb = [(bright, 0, 0), (0, bright, 0), (0, 0, bright)];
    let cmy = [
        (0, bright, bright),
        (bright, 0, bright),
        (bright, bright, 0),
    ];

    wacky_lights(
        black,
        if ticks & 0x200 == 0 { cmy } else { rgb },
        phase,
        white_idx.map(|idx| (idx, (bright, bright, bright))),
    )
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

fn wacky_lights(
    black: [(u8, u8, u8); 3],
    rgb: [(u8, u8, u8); 3],
    phase: i8,
    white_idx: Option<(u8, (u8, u8, u8))>,
) -> impl Iterator<Item = (u8, u8, u8)> {
    let mut rval = [(0, 0, 0); 12];
    for row in 0..4 {
        for col in 0..3 {
            let src = if (row as i16 - phase as i16) % 4 == 0 {
                &rgb
            } else {
                &black
            };
            let idx = row * 3 + col;
            rval[idx] = white_idx
                .and_then(|(wi, white)| {
                    if idx == wi as usize {
                        Some(white)
                    } else {
                        None
                    }
                })
                .unwrap_or(src[col]);
        }
    }

    rval.into_iter()
}
