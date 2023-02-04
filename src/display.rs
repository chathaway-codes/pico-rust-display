use embedded_graphics::{pixelcolor::Rgb565, prelude::*};

use embedded_hal::{digital::v2::OutputPin};

use display_interface_spi::SPIInterface;
use display_interface::{DataFormat, WriteOnlyDataCommand};

use embedded_graphics_core::primitives::rectangle::Rectangle;
use embedded_graphics_core::geometry::Dimensions;
use embedded_graphics_core::draw_target::DrawTarget;

const LCD_2IN_WIDTH: usize = 240;
const LCD_2IN_HEIGHT: usize = 320;

// this function was copied from LCD_2IN_Driver.c; no idea what it does.
fn init_lcd<T: WriteOnlyDataCommand >(dis: &mut T) -> Result<(), DisplayError> {
	lcd_2in_write_command(dis, 0x36)?;
	lcd_2in_write_data_byte(dis, 0x00)?;

	lcd_2in_write_command(dis, 0x3A)?; 
	lcd_2in_write_data_byte(dis, 0x05)?;

	lcd_2in_write_command(dis, 0x21)?;

	lcd_2in_write_command(dis, 0x2A)?;
	lcd_2in_write_data_byte(dis, 0x00)?;
	lcd_2in_write_data_byte(dis, 0x00)?;
	lcd_2in_write_data_byte(dis, 0x01)?;
	lcd_2in_write_data_byte(dis, 0x3F)?;

	lcd_2in_write_command(dis, 0x2B)?;
	lcd_2in_write_data_byte(dis, 0x00)?;
	lcd_2in_write_data_byte(dis, 0x00)?;
	lcd_2in_write_data_byte(dis, 0x00)?;
	lcd_2in_write_data_byte(dis, 0xEF)?;

	lcd_2in_write_command(dis, 0xB2)?;
	lcd_2in_write_data_byte(dis, 0x0C)?;
	lcd_2in_write_data_byte(dis, 0x0C)?;
	lcd_2in_write_data_byte(dis, 0x00)?;
	lcd_2in_write_data_byte(dis, 0x33)?;
	lcd_2in_write_data_byte(dis, 0x33)?;

	lcd_2in_write_command(dis, 0xB7)?;
	lcd_2in_write_data_byte(dis, 0x35)?;

	lcd_2in_write_command(dis, 0xBB)?;
	lcd_2in_write_data_byte(dis, 0x1F)?;

	lcd_2in_write_command(dis, 0xC0)?;
	lcd_2in_write_data_byte(dis, 0x2C)?;

	lcd_2in_write_command(dis, 0xC2)?;
	lcd_2in_write_data_byte(dis, 0x01)?;

	lcd_2in_write_command(dis, 0xC3)?;
	lcd_2in_write_data_byte(dis, 0x12)?;

	lcd_2in_write_command(dis, 0xC4)?;
	lcd_2in_write_data_byte(dis, 0x20)?;

	lcd_2in_write_command(dis, 0xC6)?;
	lcd_2in_write_data_byte(dis, 0x0F)?;

	lcd_2in_write_command(dis, 0xD0)?;
	lcd_2in_write_data_byte(dis, 0xA4)?;
	lcd_2in_write_data_byte(dis, 0xA1)?;

	lcd_2in_write_command(dis, 0xE0)?;
	lcd_2in_write_data_byte(dis, 0xD0)?;
	lcd_2in_write_data_byte(dis, 0x08)?;
	lcd_2in_write_data_byte(dis, 0x11)?;
	lcd_2in_write_data_byte(dis, 0x08)?;
	lcd_2in_write_data_byte(dis, 0x0C)?;
	lcd_2in_write_data_byte(dis, 0x15)?;
	lcd_2in_write_data_byte(dis, 0x39)?;
	lcd_2in_write_data_byte(dis, 0x33)?;
	lcd_2in_write_data_byte(dis, 0x50)?;
	lcd_2in_write_data_byte(dis, 0x36)?;
	lcd_2in_write_data_byte(dis, 0x13)?;
	lcd_2in_write_data_byte(dis, 0x14)?;
	lcd_2in_write_data_byte(dis, 0x29)?;
	lcd_2in_write_data_byte(dis, 0x2D)?;

	lcd_2in_write_command(dis, 0xE1)?;
	lcd_2in_write_data_byte(dis, 0xD0)?;
	lcd_2in_write_data_byte(dis, 0x08)?;
	lcd_2in_write_data_byte(dis, 0x10)?;
	lcd_2in_write_data_byte(dis, 0x08)?;
	lcd_2in_write_data_byte(dis, 0x06)?;
	lcd_2in_write_data_byte(dis, 0x06)?;
	lcd_2in_write_data_byte(dis, 0x39)?;
	lcd_2in_write_data_byte(dis, 0x44)?;
	lcd_2in_write_data_byte(dis, 0x51)?;
	lcd_2in_write_data_byte(dis, 0x0B)?;
	lcd_2in_write_data_byte(dis, 0x16)?;
	lcd_2in_write_data_byte(dis, 0x14)?;
	lcd_2in_write_data_byte(dis, 0x2F)?;
	lcd_2in_write_data_byte(dis, 0x31)?;
	lcd_2in_write_command(dis, 0x21)?;

	lcd_2in_write_command(dis, 0x11)?;

	lcd_2in_write_command(dis, 0x29)?;

    Ok(())
}

fn lcd_2in_set_window<T: WriteOnlyDataCommand >(dis: &mut T, x_start: u16, y_start: u16, x_end: u16, y_end: u16) -> Result<(), DisplayError> {
    
	lcd_2in_write_command(dis, 0x2a)?;
	lcd_2in_write_data_byte(dis, (x_start >>8) as u8)?;
	lcd_2in_write_data_byte(dis, (x_start & 0xff) as u8)?;
	lcd_2in_write_data_byte(dis, ((x_end - 1) >> 8) as u8)?;
	lcd_2in_write_data_byte(dis, ((x_end - 1) & 0xff) as u8)?;

	lcd_2in_write_command(dis, 0x2b)?;
	lcd_2in_write_data_byte(dis, (y_start >>8) as u8)?;
	lcd_2in_write_data_byte(dis, (y_start & 0xff) as u8)?;
	lcd_2in_write_data_byte(dis, ((y_end - 1) >> 8) as u8)?;
	lcd_2in_write_data_byte(dis, ((y_end - 1) & 0xff) as u8)?;

	lcd_2in_write_command(dis, 0x2C)?;

    Ok(())
}

fn lcd_2in_write_command<T: WriteOnlyDataCommand>(dis: &mut T, com: u8) -> Result<(), DisplayError> {
    dis.send_commands(DataFormat::U8(&[com])).map_err(|_| DisplayError::Unknown)?;

    Ok(())
}

fn lcd_2in_write_data_byte<T: WriteOnlyDataCommand>(dis: &mut T, com: u8) -> Result<(), DisplayError> {
    dis.send_data(DataFormat::U8(&[com])).map_err(|_| DisplayError::Unknown)?;

    Ok(())
}

#[derive(Debug)]
pub enum DisplayError{
    Unknown,
}

pub struct LCD2InchDisplay<T>{
    display: T,
}

impl<SPI, DC, CS> LCD2InchDisplay<SPIInterface<SPI, DC, CS>>
where
    SPI: embedded_hal::blocking::spi::Write<u8>,
    DC: OutputPin,
    CS: OutputPin
{
    pub fn new<RST>(spi: SPI, dc: DC, mut cs: CS, mut reset: RST, mut delay: cortex_m::delay::Delay) -> Result<Self, DisplayError>
    where
        RST: OutputPin {

        // reset the display
        cs.set_high().map_err(|_| DisplayError::Unknown)?;
        delay.delay_ms(100);
        reset.set_low().map_err(|_| DisplayError::Unknown)?;
        delay.delay_ms(100);
        reset.set_high().map_err(|_| DisplayError::Unknown)?;
        delay.delay_ms(100);

        let mut display = SPIInterface::new(spi, dc, cs);
        init_lcd(&mut display)?;
        Ok(LCD2InchDisplay{
            display: display,
        })
    }
}

impl<T> Dimensions for LCD2InchDisplay<T> {
    fn bounding_box(&self) -> Rectangle {
        Rectangle {
            top_left: Point{x:0,y:0},
            size: Size{width: LCD_2IN_WIDTH as u32, height: LCD_2IN_HEIGHT as u32}
        }
    }
}

impl<T: WriteOnlyDataCommand> DrawTarget for LCD2InchDisplay<T> {
    type Color = Rgb565;
    type Error = DisplayError;
    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>> {
            for pixel in pixels.into_iter() {
                let x: u16 = pixel.0.x as u16;
                let y: u16 = pixel.0.y as u16;
                lcd_2in_set_window(&mut self.display, x, y, x+1, y+1)?;
                self.display.send_data(DataFormat::U16BE(&mut [pixel.1.into_storage()])).map_err(|_| DisplayError::Unknown)?;
            }
            Ok(())
        }
}