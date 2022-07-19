#![no_std]

pub use adafruit_macropad::hal::clocks::init_clocks_and_plls;
pub use adafruit_macropad::hal::clocks::Clock;
pub use adafruit_macropad::hal::gpio::bank0::Gpio1;
pub use adafruit_macropad::hal::gpio::{FunctionSpi, Pin, PullUpInput, PushPullOutput};
pub use adafruit_macropad::hal::pio::PIOExt;
pub use adafruit_macropad::hal::sio::Sio;
pub use adafruit_macropad::hal::Spi;
pub use adafruit_macropad::Pins;
pub use adafruit_macropad::XOSC_CRYSTAL_FREQ;
pub use embedded_hal::spi::MODE_0;
pub use embedded_time::fixed_point::FixedPoint;
pub use embedded_time::rate::Extensions;
pub use sh1106::prelude::GraphicsMode;

#[macro_export]
macro_rules! macropad_neopixels {
    ($pins:expr, $pio:expr, $sm0: expr, $clocks:expr, $timer:expr) => {{
        ws2812_pio::Ws2812::new(
            $pins.neopixel.into_mode(),
            &mut $pio,
            $sm0,
            $crate::Clock::freq(&$clocks.peripheral_clock),
            $timer.count_down(),
        )
    }};
    ($pins: expr, $clocks:expr, $timer:expr, $pac:expr) => {{
        let (mut pio, sm0, _, _, _) = $pac.PIO0.split(&mut $pac.RESETS);
        macropad_neopixels!($pins, pio, sm0, $clocks, $timer)
    }};
}

#[macro_export]
macro_rules! macropad_oled {
    ($pins:expr, $clocks:expr, $delay: expr, $pac:expr) => {{
        let _spi_sclk = $pins.sclk.into_mode::<$crate::FunctionSpi>();
        let _spi_sclk = $pins.mosi.into_mode::<$crate::FunctionSpi>();

        let spi1 = $crate::Spi::<_, _, 8>::new($pac.SPI1).init(
            &mut $pac.RESETS,
            $crate::Clock::freq(&$clocks.peripheral_clock),
            $crate::Extensions::Hz(16_000_000u32),
            // (16_000_000u32 as embedded_time::rate::Extensions).Hz(),
            &$crate::MODE_0,
        );

        let mut oled_reset = $pins.oled_reset.into_push_pull_output();

        let mut disp: $crate::GraphicsMode<_> = sh1106::Builder::new()
            .connect_spi(
                spi1,
                $pins.oled_dc.into_push_pull_output(),
                $pins.oled_cs.into_push_pull_output(),
            )
            .into();

        disp.reset(&mut oled_reset, &mut $delay).unwrap();

        disp.init().unwrap();
        disp
    }};
}

#[macro_export]
macro_rules! macropad_pins {
    ($pac:expr) => {{
        let sio = $crate::Sio::new($pac.SIO);
        $crate::Pins::new(
            $pac.IO_BANK0,
            $pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut $pac.RESETS,
        )
    }};
}

#[macro_export]
macro_rules! macropad_clocks {
    ($pac:expr, $watchdog:expr) => {
        $crate::init_clocks_and_plls(
            $crate::XOSC_CRYSTAL_FREQ,
            $pac.XOSC,
            $pac.CLOCKS,
            $pac.PLL_SYS,
            $pac.PLL_USB,
            &mut $pac.RESETS,
            &mut $watchdog,
        )
        .ok()
        .unwrap()
    };
}

#[macro_export]
macro_rules! macropad_rotary_encoder {
    ($pins:expr) => {
        Rotary::new(
            $pins.encoder_rota.into_pull_up_input(),
            $pins.encoder_rotb.into_pull_up_input(),
        )
    };
}

#[macro_export]
macro_rules! macropad_keypad {
    ($pins:expr) => {
        KeysTwelve {
            key1: $pins.key1.into_pull_up_input(),
        }
    };
}

pub struct KeysTwelve {
    pub key1: Pin<Gpio1, PullUpInput>,
}

#[cfg(bacon_check)]
pub fn check1() {
    let mut pac = bsp::hal::pac::Peripherals::take().unwrap();
    let core = bsp::hal::pac::CorePeripherals::take().unwrap();

    let mut watchdog = adafruit_macropad::hal::Watchdog::new(pac.WATCHDOG);

    let clocks = macropad_clocks!(pac, watchdog);

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let pins = macropad_pins!(pac);

    let timer = adafruit_macropad::hal::Timer::new(pac.TIMER, &mut pac.RESETS);

    //

    let mut disp = macropad_oled!(pins, clocks, delay, pac);

    let mut led_pin = pins.led.into_push_pull_output();

    let mut neopixels = macropad_neopixels!(pins, clocks, timer, pac);
}
