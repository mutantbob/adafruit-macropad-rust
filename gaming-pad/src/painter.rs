use crate::{ChangeDetector, UfmtWrapper};
use embedded_graphics::drawable::Drawable;
use embedded_graphics::fonts::{Font6x8, Text};
use embedded_graphics::style::TextStyle;
use embedded_graphics_core::geometry::{Point, Size};
use embedded_graphics_core::pixelcolor::BinaryColor;
use embedded_graphics_core::primitives::Rectangle;
use macropad_helpers::DTWrapper;
use ufmt::uwrite;

pub fn easy_text_at<E, D>(
    msg: &str,
    x: i32,
    y: i32,
    disp: &mut D,
    color: embedded_graphics::pixelcolor::BinaryColor,
) -> Result<(), E>
where
    D: embedded_graphics_core::draw_target::DrawTarget<
        Color = embedded_graphics_core::pixelcolor::BinaryColor,
        Error = E,
    >,
{
    Text::new(msg, embedded_graphics::geometry::Point::new(x, y))
        .into_styled(TextStyle::new(Font6x8, color))
        .draw(&mut DTWrapper { inner: disp })
}

fn paint_multiline_text<D, E>(
    x: i32,
    mut y: i32,
    mut msg: &str,
    fg: embedded_graphics::pixelcolor::BinaryColor,
    bg: BinaryColor,
    disp: &mut D,
) -> Result<(), E>
where
    D: embedded_graphics_core::draw_target::DrawTarget<
        Color = embedded_graphics_core::pixelcolor::BinaryColor,
        Error = E,
    >,
{
    loop {
        let ab = msg.split_once('\n');
        let (submessage, more) = match ab {
            Some((a, b)) => (a, Some(b)),
            None => (msg, None),
        };
        const DISPLAY_WIDTH: i32 = 128;
        disp.fill_solid(
            &Rectangle::new(Point::new(x, y), Size::new((DISPLAY_WIDTH - x) as u32, 10)),
            bg,
        )?;

        easy_text_at(submessage, x, y + 1, disp, fg)?;
        match more {
            None => {
                break;
            }
            Some(more) => {
                msg = more;
                y += 9;
            }
        }
    }
    Ok(())
}

//

pub struct DisplayPainter<D, E>
where
    D: embedded_graphics_core::draw_target::DrawTarget<
        Color = embedded_graphics_core::pixelcolor::BinaryColor,
        Error = E,
    >,
{
    pub disp: D,
    old_brightness: ChangeDetector<u8>,
    old_key_state: ChangeDetector<[bool; 12]>,
}

impl<D, E> DisplayPainter<D, E>
where
    D: embedded_graphics_core::draw_target::DrawTarget<
        Color = embedded_graphics_core::pixelcolor::BinaryColor,
        Error = E,
    >,
{
    pub fn new(disp: D) -> Self {
        DisplayPainter {
            disp,
            old_brightness: ChangeDetector::new(0),
            old_key_state: ChangeDetector::new([true; 12]),
        }
    }

    pub fn idle_display(&mut self) -> Result<(), E> {
        const LABELS: [[&str; 3]; 4] = [
            ["sprint", "mine", "   -   "],
            ["jog 70%", "   -   ", "   -   "],
            ["jog 60%", "   -   ", "   -   "],
            ["walk", "   -   ", "   -   "],
        ];
        self.disp
            .clear(embedded_graphics_core::pixelcolor::BinaryColor::Off)?;
        for (row, labels) in LABELS.iter().enumerate() {
            for (col, label) in labels.iter().enumerate() {
                let x = (128 + 1) * col / 3;
                let y = 10 * row;
                easy_text_at(
                    *label,
                    x as i32,
                    y as i32,
                    &mut self.disp,
                    embedded_graphics::pixelcolor::BinaryColor::On,
                )?;
            }
        }
        Ok(())
    }

    pub fn update_display(
        &mut self,
        bright: u8,
        key_state: [bool; 12],
        active_mission: Option<u8>,
        debug_msg: &str,
    ) -> Result<(), E> {
        if self.old_brightness.changed(bright) {
            self.paint_brightness()?;
        }

        if self.old_key_state.changed(key_state) {
            self.paint_key0_state(key_state)?;
        }

        if true {
            return Ok(());
        }

        self.paint_key_grid(active_mission)?;

        if !debug_msg.is_empty() {
            paint_multiline_text(
                0,
                10,
                debug_msg,
                embedded_graphics::pixelcolor::BinaryColor::On,
                BinaryColor::Off,
                &mut self.disp,
            )?;
        }

        Ok(())
    }

    fn paint_key_grid(&mut self, active_mission: Option<u8>) -> Result<(), E> {
        let diam = 5;
        for row in 0..4 {
            for col in 0..3 {
                let idx = row * 3 + col;
                self.disp.fill_solid(
                    &Rectangle::new(
                        Point::new(col * diam, row * diam),
                        Size::new(diam as u32, diam as u32),
                    ),
                    if active_mission == Some(idx as u8) {
                        BinaryColor::On
                    } else {
                        BinaryColor::Off
                    },
                )?;
            }
        }
        Ok(())
    }

    fn paint_key0_state(&mut self, key_state: [bool; 12]) -> Result<(), E> {
        let p1 = Point::new(20, 1);
        self.disp
            .fill_solid(&Rectangle::new(p1, Size::new(100, 10)), BinaryColor::Off)?;
        easy_text_at(
            if key_state[0] { "0down" } else { "0up" },
            p1.x + 1,
            p1.y + 1,
            &mut self.disp,
            embedded_graphics::pixelcolor::BinaryColor::On,
        )
    }

    fn paint_brightness(&mut self) -> Result<(), E> {
        let p1 = Point::new(20, 40);
        self.disp
            .fill_solid(&Rectangle::new(p1, Size::new(100, 60)), BinaryColor::On)?;

        let mut fmt_buffer = UfmtWrapper::<80>::new();

        let bright = self.old_brightness.old;
        // fmt_buffer.write_str("bright");
        uwrite!(&mut fmt_buffer, "brightness={}", bright).unwrap();

        easy_text_at(
            fmt_buffer.as_str(),
            // "brightness",
            p1.x + 1,
            p1.y + 1,
            &mut self.disp,
            embedded_graphics::pixelcolor::BinaryColor::Off,
        )
    }
}
