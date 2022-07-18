// use adafruit_macropad::hal::clocks::Clock;
// use adafruit_macropad::hal::gpio::PushPullOutput;
// use adafruit_macropad::hal::gpio::{FunctionSpi, Input, PullUp};
// use adafruit_macropad::hal::pio::PIOExt;
// use adafruit_macropad::XOSC_CRYSTAL_FREQ;
// use embedded_time::fixed_point::FixedPoint;
// use embedded_time::rate::Extensions;

macro_rules! macropad_neopixels {
    ($pins:expr, $pio:expr, $sm0: expr, $clocks:expr, $timer:expr) => {{
        ws2812_pio::Ws2812::new(
            $pins.neopixel.into_mode(),
            &mut $pio,
            $sm0,
            adafruit_macropad::hal::clocks::Clock::freq(&$clocks.peripheral_clock),
            $timer.count_down(),
        )
    }};
    ($pins: expr, $clocks:expr, $timer:expr, $pac:expr) => {{
        let (mut pio, sm0, _, _, _) = $pac.PIO0.split(&mut $pac.RESETS);
        macropad_neopixels!($pins, pio, sm0, $clocks, $timer)
    }};
}

macro_rules! macropad_oled {
    ($pins:expr, $clocks:expr, $delay: expr, $pac:expr) => {{
        let _spi_sclk = $pins
            .sclk
            .into_mode::<adafruit_macropad::hal::gpio::FunctionSpi>();
        let _spi_sclk = $pins
            .mosi
            .into_mode::<adafruit_macropad::hal::gpio::FunctionSpi>();

        let spi1 = adafruit_macropad::hal::Spi::<_, _, 8>::new($pac.SPI1).init(
            &mut $pac.RESETS,
            adafruit_macropad::hal::clocks::Clock::freq(&$clocks.peripheral_clock),
            embedded_time::rate::Extensions::Hz(16_000_000u32),
            // (16_000_000u32 as embedded_time::rate::Extensions).Hz(),
            &embedded_hal::spi::MODE_0,
        );

        let mut oled_reset = $pins
            .oled_reset
            .into_mode::<adafruit_macropad::hal::gpio::PushPullOutput>();

        let mut disp: sh1106::prelude::GraphicsMode<_> = sh1106::Builder::new()
            .connect_spi(
                spi1,
                $pins
                    .oled_dc
                    .into_mode::<adafruit_macropad::hal::gpio::PushPullOutput>(),
                $pins
                    .oled_cs
                    .into_mode::<adafruit_macropad::hal::gpio::PushPullOutput>(),
            )
            .into();

        disp.reset(&mut oled_reset, &mut $delay).unwrap();

        disp.init().unwrap();
        disp
    }};
}

macro_rules! macropad_pins {
    ($pac:expr) => {{
        let sio = crate::bsp::hal::sio::Sio::new($pac.SIO);
        adafruit_macropad::Pins::new(
            $pac.IO_BANK0,
            $pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut $pac.RESETS,
        )
    }};
}

macro_rules! macropad_clocks {
    ($pac:expr, $watchdog:expr) => {
        crate::bsp::hal::clocks::init_clocks_and_plls(
            adafruit_macropad::XOSC_CRYSTAL_FREQ,
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

#[cfg(bacon_check)]
pub fn check1() {
    let mut pac = crate::bsp::hal::pac::Peripherals::take().unwrap();
    let core = crate::bsp::hal::pac::CorePeripherals::take().unwrap();

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
