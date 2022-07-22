use crate::{ChangeDetector, UfmtWrapper};
use bitmap_font::tamzen::FONT_8x15;
use bitmap_font::TextStyle;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::{Point, Size};
use embedded_graphics::primitives::Rectangle;
use embedded_graphics::text::renderer::TextRenderer;
use embedded_graphics::text::Text;
use embedded_graphics_core::Drawable;
use embedded_vintage_fonts::FONT_6X8;
use ufmt::uwrite;

pub fn easy_text_at<C, D, E, S>(
    msg: &str,
    x: i32,
    y: i32,
    disp: &mut D,
    style: S,
) -> Result<Point, E>
where
    D: embedded_graphics_core::prelude::DrawTarget<Color = C, Error = E>,
    S: TextRenderer<Color = C>,
{
    Text::new(msg, embedded_graphics::geometry::Point::new(x, y), style).draw(
        disp, // &mut DTWrapper { inner: disp }
    )
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

        easy_text_at(
            submessage,
            x,
            y + 1,
            disp,
            MonoTextStyle::new(&FONT_6X8, fg),
        )?;
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
    pub dirty: bool,
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
            dirty: true,
            disp,
            old_brightness: ChangeDetector::new(0),
            old_key_state: ChangeDetector::new([true; 12]),
        }
    }

    pub fn idle_display(&mut self) -> Result<(), E> {
        if !self.dirty {
            return Ok(());
        }

        const LABELS: [[&str; 3]; 4] = [
            ["sprint", "mine", "-"],
            ["jog 70%", "-", "-"],
            ["jog 60%", "-", "-"],
            ["walk", "-", "-"],
        ];
        self.disp
            .clear(embedded_graphics_core::pixelcolor::BinaryColor::Off)?;

        easy_text_at(
            "Bob's Macro Pad",
            1,
            0,
            &mut self.disp,
            TextStyle::new(
                &FONT_8x15,
                embedded_graphics_core::pixelcolor::BinaryColor::On,
            ),
        )?;

        for (row, labels) in LABELS.iter().enumerate() {
            for (col, label) in labels.iter().enumerate() {
                let x = (128 + 1) * col / 3 + 21 - label.len() * 3;
                let y = 10 * row + 25;
                easy_text_at(
                    *label,
                    x as i32,
                    y as i32,
                    &mut self.disp,
                    MonoTextStyle::new(
                        &FONT_6X8,
                        embedded_graphics_core::pixelcolor::BinaryColor::On,
                    ),
                )?;
            }
        }
        self.dirty = false;
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

    fn paint_key0_state(&mut self, key_state: [bool; 12]) -> Result<Point, E> {
        let p1 = Point::new(20, 1);
        self.disp
            .fill_solid(&Rectangle::new(p1, Size::new(100, 10)), BinaryColor::Off)?;
        easy_text_at(
            if key_state[0] { "0down" } else { "0up" },
            p1.x + 1,
            p1.y + 1,
            &mut self.disp,
            MonoTextStyle::new(
                &FONT_6X8,
                embedded_graphics_core::pixelcolor::BinaryColor::On,
            ),
        )
    }

    fn paint_brightness(&mut self) -> Result<Point, E> {
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
            MonoTextStyle::new(
                &FONT_6X8,
                embedded_graphics_core::pixelcolor::BinaryColor::On,
            ),
        )
    }
}
