use usbd_hid::descriptor::{KeyboardReport, MouseReport};

pub fn simple_kr(modifier: u8, keycodes: [u8; 6]) -> KeyboardReport {
    KeyboardReport {
        modifier,
        reserved: 0,
        leds: 0,
        keycodes,
    }
}

pub fn simple_kr1(modifier: u8, key_code_1: u8) -> KeyboardReport {
    simple_kr(modifier, [key_code_1, 0, 0, 0, 0, 0])
}

pub trait MissionMode<T> {
    fn reboot(&mut self);

    fn one_usb_pass(&mut self, millis_elapsed: u32) -> T;

    fn maybe_deactivate(&mut self) -> Option<T>;
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
}

impl MouseHold {
    pub fn new() -> Self {
        MouseHold { deactivated: false }
    }
}

impl MissionMode<KeyboardOrMouse> for MouseHold {
    fn reboot(&mut self) {
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
