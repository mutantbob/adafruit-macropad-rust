use embedded_graphics::drawable::Drawable;
use embedded_graphics::fonts::{Font6x8, Text};
use embedded_graphics::geometry::Size;
use embedded_graphics::primitives::Line;
use embedded_graphics::style::{PrimitiveStyle, Styled, TextStyle};
use sh1106::interface::DisplayInterface;
use sh1106::mode::GraphicsMode;

pub struct DTWrapper<
    'a,
    E,
    D: embedded_graphics_core::draw_target::DrawTarget<
        Color = embedded_graphics_core::pixelcolor::BinaryColor,
        Error = E,
    >,
> {
    pub inner: &'a mut D,
}

impl<
        'a,
        E,
        D: embedded_graphics_core::draw_target::DrawTarget<
            Color = embedded_graphics_core::pixelcolor::BinaryColor,
            Error = E,
        >,
    > embedded_graphics::DrawTarget<embedded_graphics::pixelcolor::BinaryColor>
    for DTWrapper<'a, E, D>
{
    type Error = E;

    fn draw_pixel(
        &mut self,
        item: embedded_graphics::drawable::Pixel<embedded_graphics::pixelcolor::BinaryColor>,
    ) -> Result<(), Self::Error> {
        let point: embedded_graphics_core::geometry::Point =
            embedded_graphics_core::geometry::Point::new(item.0.x, item.0.y);
        let color = if item.1.is_on() {
            embedded_graphics_core::pixelcolor::BinaryColor::On
        } else {
            embedded_graphics_core::pixelcolor::BinaryColor::Off
        };
        let p: embedded_graphics_core::Pixel<embedded_graphics_core::pixelcolor::BinaryColor> =
            embedded_graphics_core::Pixel(point, color);
        self.inner.draw_iter(core::iter::once(p))
    }

    fn size(&self) -> Size {
        let bounds = self.inner.bounding_box();
        Size::new(bounds.size.width, bounds.size.height)
    }
}

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
