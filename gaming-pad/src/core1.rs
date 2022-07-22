use adafruit_macropad::hal::sio::SioFifo;
use adafruit_macropad::{pac, KeysTwelve, Sio};
use embedded_hal::digital::v2::InputPin;
use rotary_encoder_hal::{Direction, Rotary};

struct Core1Application<RotA, RotB>
where
    RotA: InputPin,
    RotB: InputPin,
{
    rotary: Rotary<RotA, RotB>,
    keys: KeysTwelve,

    rotary_delta: i32,
    fire_idx: i32,
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
            rotary_delta: 0,
            fire_idx: -1,
            key_state: [false; 12],
            old_key_state: [false; 12],
        }
    }

    pub fn one_pass(&mut self, fifo: &mut SioFifo) {
        match self.rotary.update() {
            Ok(Direction::Clockwise) => {
                self.rotary_delta += 1;
            }
            Ok(Direction::CounterClockwise) => {
                self.rotary_delta -= 1;
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
            *fire = self.key_state[idx] && !self.old_key_state[idx];
        }

        let fire_idx = fire_array
            .iter()
            .enumerate()
            .find(|(_, b)| **b)
            .map(|(idx, _)| idx as i32);

        if let Some(fire_idx) = fire_idx {
            self.fire_idx = fire_idx;
        }

        //

        self.old_key_state = self.key_state;

        self.maybe_send_state(fifo);
    }

    fn maybe_send_state(&mut self, fifo: &mut SioFifo) {
        let msg = fifo.read();
        if let Some(msg) = msg {
            if 0 == msg {
                fifo.write_blocking(self.rotary_delta as u32);
                self.rotary_delta = 0;
                fifo.write_blocking(self.fire_idx as u32);
                self.fire_idx = -1;
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
