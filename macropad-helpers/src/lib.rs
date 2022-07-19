#![no_std]

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

    fn size(&self) -> embedded_graphics::prelude::Size {
        let bounds = self.inner.bounding_box();
        embedded_graphics::prelude::Size::new(bounds.size.width, bounds.size.height)
    }
}
