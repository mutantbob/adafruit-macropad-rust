use embedded_graphics::drawable::Drawable;
use embedded_graphics::fonts::{Font6x8, Text};
use embedded_graphics::primitives::Line;
use embedded_graphics::style::{PrimitiveStyle, Styled, TextStyle};
use macropad_helpers::DTWrapper;
use sh1106::interface::DisplayInterface;
use sh1106::mode::GraphicsMode;

pub fn try_to_draw<DI: DisplayInterface>(disp: &mut GraphicsMode<DI>) {
    use embedded_graphics::pixelcolor::BinaryColor;
    use embedded_graphics::prelude::Point;
    let wrapper = &mut DTWrapper { inner: disp };
    Text::new("hail Bob!", Point::zero())
        .into_styled(TextStyle::new(Font6x8, BinaryColor::On))
        .draw(wrapper)
        .unwrap();

    Text::new("or die", Point::new(5, 10))
        .into_styled(TextStyle::new(Font6x8, BinaryColor::On))
        .draw(wrapper)
        .unwrap();

    Styled::new(
        Line::new(Point::new(5, 35), Point::new(30, 40)),
        PrimitiveStyle::with_stroke(BinaryColor::On, 2),
    )
    .draw(wrapper)
    .unwrap();

    Styled::new(
        Line::new(Point::new(5, 40), Point::new(30, 50)),
        PrimitiveStyle::with_stroke(BinaryColor::On, 2),
    )
    .draw(wrapper)
    .unwrap();

    let _ = disp.flush();
}
