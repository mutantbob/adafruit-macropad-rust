use adafruit_macropad::hal::sio::SioFifo;
use adafruit_macropad::{pac, KeysTwelve, Sio};
use embedded_hal::digital::v2::InputPin;
use rotary_encoder_hal::{Direction, Rotary};

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum JogAxis {
    X,
    Y,
    Z,
}

impl JogAxis {
    pub fn as_int(&self) -> u32 {
        match self {
            JogAxis::X => 0,
            JogAxis::Y => 1,
            JogAxis::Z => 2,
        }
    }
}

impl TryFrom<u32> for JogAxis {
    type Error = ();

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(JogAxis::X),
            1 => Ok(JogAxis::Y),
            2 => Ok(JogAxis::Z),
            _ => Err(()),
        }
    }
}

#[derive(Debug, Copy, Clone)]
#[allow(upper_case_acronyms)]
pub enum LastJog {
    None,
    CW,
    CCW,
}

impl LastJog {
    pub fn as_int(&self) -> u32 {
        match self {
            LastJog::None => 0,
            LastJog::CW => 1,
            LastJog::CCW => 2,
        }
    }
}

impl TryFrom<u32> for LastJog {
    type Error = ();

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(LastJog::None),
            1 => Ok(LastJog::CW),
            2 => Ok(LastJog::CCW),
            _ => Err(()),
        }
    }
}

//

struct Core1Application<RotA, RotB>
where
    RotA: InputPin,
    RotB: InputPin,
{
    rotary: Rotary<RotA, RotB>,
    keys: KeysTwelve,

    jog_axis: JogAxis,
    last_jog: LastJog,
    last_key: Option<u8>,
    key_state: [bool; 12],
    old_key_state: [bool; 12],
}

impl<RotA, RotB> Core1Application<RotA, RotB>
where
    RotA: InputPin,
    RotB: InputPin,
{
    pub fn new(rotary: Rotary<RotA, RotB>, keys: KeysTwelve) -> Self {
        Core1Application {
            rotary,
            keys,
            jog_axis: JogAxis::X,
            last_jog: LastJog::None,
            last_key: None,
            key_state: [false; 12],
            old_key_state: [false; 12],
        }
    }

    pub fn one_pass(&mut self, fifo: &mut SioFifo) {
        self.last_jog = match self.rotary.update() {
            Ok(Direction::Clockwise) => LastJog::CW,
            Ok(Direction::CounterClockwise) => LastJog::CCW,
            _ => LastJog::None,
        };

        {
            let keys_array = self.keys.array_0based();
            for (idx, state) in self.key_state.iter_mut().enumerate() {
                let pressed = keys_array[idx].is_low().unwrap();
                *state = pressed;
            }
        }

        let recently_pressed = self
            .key_state
            .iter()
            .zip(self.old_key_state.iter())
            .enumerate()
            .find(|(_, (fresh, old))| **fresh && !**old)
            .map(|(idx, _)| idx as u8);

        let recently_pressed = recently_pressed.or_else(|| {
            if let Some(idx) = self.last_key {
                if self.key_state[idx as usize] {
                    return Some(idx);
                }
            }
            self.key_state
                .iter()
                .enumerate()
                .filter_map(|(idx, fresh)| if *fresh { Some(idx as u8) } else { None })
                .next()
        });

        match self.last_key {
            Some(idx) => {
                self.last_key = recently_pressed;
            }
            None => {
                self.last_key = recently_pressed;
            }
        }

        match self.last_key {
            Some(9) => self.jog_axis = JogAxis::X,
            Some(10) => self.jog_axis = JogAxis::Y,
            Some(11) => self.jog_axis = JogAxis::Z,
            _ => {}
        }

        //

        self.maybe_send_state(fifo);

        self.old_key_state = self.key_state;
    }

    fn maybe_send_state(&mut self, fifo: &mut SioFifo) {
        let msg = fifo.read();
        if let Some(msg) = msg {
            if 0 == msg {
                InterCoreMessage {
                    jog_axis: self.jog_axis,
                    jog_action: self.last_jog,
                    last_key: self.last_key,
                }
                .send(fifo);
            }
        }
    }
}

pub fn core1_task<ROTA, ROTB>(rotary: Rotary<ROTA, ROTB>, keys: KeysTwelve) -> !
where
    ROTA: InputPin,
    ROTB: InputPin,
{
    let pac = unsafe { pac::Peripherals::steal() };
    // let core = unsafe { pac::CorePeripherals::steal() };
    let mut sio = Sio::new(pac.SIO);

    let mut app = Core1Application::new(rotary, keys);

    //

    loop {
        app.one_pass(&mut sio.fifo);
    }
}

pub struct InterCoreMessage {
    pub jog_axis: JogAxis,
    pub jog_action: LastJog,
    pub last_key: Option<u8>,
}

impl InterCoreMessage {
    pub fn send(&self, fifo: &mut SioFifo) {
        fifo.write_blocking(self.jog_axis.as_int());

        fifo.write_blocking(self.jog_action.as_int());

        let last_key = self.last_key.map(|x| x as u32).unwrap_or(u32::MAX) as u32;
        fifo.write_blocking(last_key);
    }

    pub fn receive(fifo: &mut SioFifo) -> Result<Self, ()> {
        let jog_axis: JogAxis = fifo.read_blocking().try_into()?;
        let jog_action: LastJog = fifo.read_blocking().try_into()?;
        let raw = fifo.read_blocking();
        let last_key = match raw {
            0..=11 => Some(raw as u8),
            _ => None,
        };

        Ok(Self {
            jog_axis,
            last_key,
            jog_action,
        })
    }
}
