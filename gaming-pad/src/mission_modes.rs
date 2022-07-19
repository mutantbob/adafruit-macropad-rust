use keycode_translation::{simple_kr1, CodeSequence};
use usbd_hid::descriptor::{KeyboardReport, MouseReport};

pub trait MissionMode<T> {
    fn reboot(&mut self);

    fn one_usb_pass(&mut self, millis_elapsed: u32) -> T;

    fn maybe_deactivate(&mut self) -> Option<T>;
}

//

pub struct Ia {
    generator: CodeSequence<core::iter::Cycle<core::str::Chars<'static>>>,
    deactivated: bool,
}

impl Ia {
    pub fn standard_generator() -> CodeSequence<core::iter::Cycle<core::str::Chars<'static>>> {
        let chars = "Ia! Ia! Cthulhu fhtagn.  ".chars();
        CodeSequence::new(chars.cycle())
    }
}

impl Default for Ia {
    fn default() -> Self {
        Ia {
            generator: Self::standard_generator(),
            deactivated: false,
        }
    }
}

impl MissionMode<KeyboardReport> for Ia {
    fn reboot(&mut self) {
        self.generator = Self::standard_generator();
    }

    fn one_usb_pass(&mut self, _millis_elapsed: u32) -> KeyboardReport {
        self.generator.next().unwrap()
    }

    fn maybe_deactivate(&mut self) -> Option<KeyboardReport> {
        if self.deactivated {
            None
        } else {
            self.deactivated = true;
            Some(simple_kr1(0, 0))
        }
    }
}

//
/*
pub struct CallOfCthulhu {
    generator: CodeSequence<'static>,
    deactivated: bool,
}

impl CallOfCthulhu {
    fn story_text() -> impl Iterator<Item = char> {
        let orig = include_bytes!("../keycode_translation/src/call-of-cthulhu.txt");
        orig.iter().cycle().map(|&b| b as char)
    }

    fn standard_generator() -> CodeSequence<'static> {
        CodeSequence::new(Box::new(Self::story_text()))
    }
}

impl Default for CallOfCthulhu {
    fn default() -> Self {
        CallOfCthulhu {
            generator: Self::standard_generator(),
            deactivated: false,
        }
    }
}

impl MissionMode<KeyboardReport> for CallOfCthulhu {
    fn reboot(&mut self) {
        self.generator = Self::standard_generator();
    }

    fn one_usb_pass(&mut self, _millis_elapsed: u32) -> KeyboardReport {
        self.generator.next().unwrap()
    }

    fn maybe_deactivate(&mut self) -> Option<KeyboardReport> {
        if self.deactivated {
            None
        } else {
            self.deactivated = true;
            Some(simple_kr1(0, 0))
        }
    }
}
*/
//

#[derive(Default)]
pub struct Eeeeee {
    deactivated: bool,
}

impl MissionMode<KeyboardReport> for Eeeeee {
    fn reboot(&mut self) {}

    fn one_usb_pass(&mut self, _millis_elapsed: u32) -> KeyboardReport {
        const CODE: u8 = b'e' - b'a' + 4;
        simple_kr1(0, CODE)
    }

    fn maybe_deactivate(&mut self) -> Option<KeyboardReport> {
        if self.deactivated {
            None
        } else {
            self.deactivated = true;
            Some(simple_kr1(0, 0))
        }
    }
}

//

pub enum KeyboardOrMouse {
    Keyboard(KeyboardReport),
    Mouse(MouseReport),
}

//

pub struct IcarusJog {
    sprint: bool,
    duty_cycle: f32,
    deactivated: bool,
    phase_millis: Option<u32>,
}

impl IcarusJog {
    pub fn new(duty_cycle: f32) -> IcarusJog {
        IcarusJog {
            sprint: false,
            duty_cycle,
            deactivated: false,
            phase_millis: None,
        }
    }
}

impl MissionMode<KeyboardOrMouse> for IcarusJog {
    fn reboot(&mut self) {
        self.sprint = false;
        self.phase_millis = None;
    }

    fn one_usb_pass(&mut self, millis_elapsed: u32) -> KeyboardOrMouse {
        let phase_millis = match self.phase_millis {
            None => {
                self.phase_millis = Some(millis_elapsed);
                millis_elapsed
            }
            Some(val) => val,
        };
        let period = 6000;
        self.sprint =
            ((millis_elapsed - phase_millis) % period) < (period as f32 * self.duty_cycle) as u32;

        //support::time_elapse(gpt1, || self.sprint = !self.sprint);
        /*let mut status = gpt2.output_compare_status(hal::gpt::OutputCompareRegister::One);
        if status.is_set() {
            status.clear();
            self.sprint = !self.sprint
        }*/

        let keyboard_report = simple_kr1(if self.sprint { 2 } else { 0 }, b'w' - b'a' + 4);
        KeyboardOrMouse::Keyboard(keyboard_report)
    }

    fn maybe_deactivate(&mut self) -> Option<KeyboardOrMouse> {
        if self.deactivated {
            None
        } else {
            self.deactivated = true;
            Some(KeyboardOrMouse::Keyboard(simple_kr1(0, 0)))
        }
    }
}

//

pub struct MouseHold {
    deactivated: bool,
    down: bool,
}

impl MouseHold {
    pub fn new() -> Self {
        MouseHold {
            deactivated: false,
            down: false,
        }
    }
}

impl MissionMode<KeyboardOrMouse> for MouseHold {
    fn reboot(&mut self) {
        self.down = false;
        self.deactivated = false;
    }

    fn one_usb_pass(&mut self, _millis_elapsed: u32) -> KeyboardOrMouse {
        KeyboardOrMouse::Mouse(MouseReport {
            buttons: 1,
            x: 0,
            y: 0,
            wheel: 0,
            pan: 0,
        })
    }

    fn maybe_deactivate(&mut self) -> Option<KeyboardOrMouse> {
        if self.deactivated {
            None
        } else {
            self.deactivated = true;
            Some(KeyboardOrMouse::Mouse(MouseReport {
                buttons: 0,
                x: 0,
                y: 0,
                wheel: 0,
                pan: 0,
            }))
        }
    }
}

//

pub struct TestKey {
    key_code: KeyboardReport,
    deactivated: bool,
}

impl TestKey {
    pub fn new(ch: char) -> TestKey {
        TestKey {
            key_code: keycode_translation::translate_char(ch).unwrap(),
            deactivated: false,
        }
    }
}

impl MissionMode<KeyboardReport> for TestKey {
    fn reboot(&mut self) {
        // nothing to do
    }

    fn one_usb_pass(&mut self, _millis_elapsed: u32) -> KeyboardReport {
        self.key_code
    }

    fn maybe_deactivate(&mut self) -> Option<KeyboardReport> {
        if self.deactivated {
            None
        } else {
            self.deactivated = true;
            Some(simple_kr1(0, 0))
        }
    }
}
