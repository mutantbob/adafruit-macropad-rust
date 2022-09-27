use usbd_hid::descriptor::KeyboardReport;

pub const LEFT_CTRL: u8 = 1;
#[allow(dead_code)]
pub const LEFT_SHIFT: u8 = 2;
pub const LEFT_ALT: u8 = 4;
#[allow(dead_code)]
pub const LEFT_META: u8 = 8;

pub const KP_PAGE_UP: u8 = 0x4b;
pub const KP_PAGE_DOWN: u8 = 0x4e;
pub const KP_RIGHT_ARROW: u8 = 0x4f;
pub const KP_LEFT_ARROW: u8 = 0x50;
pub const KP_DOWN_ARROW: u8 = 0x51;
pub const KP_UP_ARROW: u8 = 0x52;

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

fn number_keycode(digit: u8) -> u8 {
    match digit {
        0 => 0x27,
        _ => 0x1d + digit,
    }
}

//

pub fn key_z_minus() -> KeyboardReport {
    simple_kr1(LEFT_CTRL | LEFT_ALT, KP_PAGE_DOWN)
}

pub fn key_z_plus() -> KeyboardReport {
    simple_kr1(LEFT_CTRL | LEFT_ALT, KP_PAGE_UP)
}

pub fn key_y_minus() -> KeyboardReport {
    simple_kr1(LEFT_CTRL | LEFT_ALT, KP_DOWN_ARROW)
}

pub fn key_x_plus() -> KeyboardReport {
    simple_kr1(LEFT_CTRL | LEFT_ALT, KP_RIGHT_ARROW)
}

pub fn key_x_minus() -> KeyboardReport {
    simple_kr1(LEFT_CTRL | LEFT_ALT, KP_LEFT_ARROW)
}

pub fn key_y_plus() -> KeyboardReport {
    simple_kr1(LEFT_CTRL | LEFT_ALT, KP_UP_ARROW)
}

pub fn key_jog_scale(jog_scale: u8) -> KeyboardReport {
    simple_kr1(LEFT_CTRL | LEFT_ALT, number_keycode(1 + jog_scale))
}
