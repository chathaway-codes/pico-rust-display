// Turn off std and main; they assume some features are available
// which aren't (i.e., a heap).
#![no_std]
#![no_main]

// We need to define our own panic handler
use core::panic::PanicInfo;
use cortex_m::delay::Delay;

use embedded_graphics::{image::Image, pixelcolor::Rgb565, prelude::*};
use tinybmp::Bmp;

use embedded_time::rate::*;
// peripheral access crate
//use rp_pico::hal::{self, pac};

use embedded_hal::spi::MODE_0;
use fugit::RateExtU32;
use rp2040_hal::{spi::Spi, pac, gpio::{bank0::Gpio12, Pin, Pins, PushPullOutput}, sio::Sio};

use embedded_hal::{digital::v2::OutputPin, blocking::spi::Write};
use rp_pico::{
    self,
    hal::{self, prelude::*},
};

use display_interface_spi::SPIInterface;
use display_interface::WriteOnlyDataCommand;
use display_interface::DataFormat;

use embedded_graphics_core::primitives::rectangle::Rectangle;
use embedded_graphics_core::geometry::Dimensions;
use embedded_graphics_core::draw_target::DrawTarget;

// The macro for our start-up function
use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    // take the peripherals
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    
    // Watchdog is a HW timer that counts down, and if it
    // reaches 0, restarts the program. We use this as a timer
    // when creating our clock.
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();
    // create the delay
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().raw());

    // get access to the GPIO pins
    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let _ = pins.gpio2.into_mode::<rp2040_hal::gpio::FunctionSpi>();
    let _ = pins.gpio3.into_mode::<rp2040_hal::gpio::FunctionSpi>();
    // miso is rx; recv data from client. since its a display, it never sends data
    let _ = pins.gpio4.into_mode::<rp2040_hal::gpio::FunctionSpi>();

    let spi = Spi::<_, _, 8>::new(pac.SPI0).init(&mut pac.RESETS, RateExtU32::Hz(125_000_000u32), RateExtU32::Hz(125_000_000u32), &MODE_0);
    let dc = pins.gpio6.into_push_pull_output();
    let mut cs = pins.gpio5.into_push_pull_output();
    let mut reset = pins.gpio16.into_push_pull_output();

    // reset the display
    cs.set_high().unwrap();
    delay.delay_ms(100);
    reset.set_low().unwrap();
    delay.delay_ms(100);
    reset.set_high().unwrap();
    delay.delay_ms(100);

    let mut display = SPIInterface::new(spi, dc, cs);

    // display is 240x320; assuming 24 bit colors.
    let display_buffer = [0xAA; 240*320*3];

    init_lcd(&mut display);
    //LCD_2IN_Clear(&mut display, 0xFFFF);
    let bmp_data = include_bytes!("../image.bmp");
    let bmp: Bmp<Rgb565> = Bmp::from_slice(bmp_data).unwrap();
    let image = Image::new(&bmp, Point::new(0, 0));

    let mut display = LCD2InchDisplay{
        display: display,
    };
    image.draw(&mut display);

    //image.draw(&mut display);

    //blit_iterator(&mut display, bmp.pixels());

    loop {
        //set_window(&mut display, 100, 20, 100, 20);
        //write_color(&mut display, 0xAA);
    }
}

fn init_lcd<T: WriteOnlyDataCommand >(dis: &mut T) {
    

	LCD_2IN_Write_Command(dis, 0x36);
	LCD_2IN_WriteData_Byte(dis, 0x00); 

	LCD_2IN_Write_Command(dis, 0x3A); 
	LCD_2IN_WriteData_Byte(dis, 0x05);

	LCD_2IN_Write_Command(dis, 0x21); 

	LCD_2IN_Write_Command(dis, 0x2A);
	LCD_2IN_WriteData_Byte(dis, 0x00);
	LCD_2IN_WriteData_Byte(dis, 0x00);
	LCD_2IN_WriteData_Byte(dis, 0x01);
	LCD_2IN_WriteData_Byte(dis, 0x3F);

	LCD_2IN_Write_Command(dis, 0x2B);
	LCD_2IN_WriteData_Byte(dis, 0x00);
	LCD_2IN_WriteData_Byte(dis, 0x00);
	LCD_2IN_WriteData_Byte(dis, 0x00);
	LCD_2IN_WriteData_Byte(dis, 0xEF);

	LCD_2IN_Write_Command(dis, 0xB2);
	LCD_2IN_WriteData_Byte(dis, 0x0C);
	LCD_2IN_WriteData_Byte(dis, 0x0C);
	LCD_2IN_WriteData_Byte(dis, 0x00);
	LCD_2IN_WriteData_Byte(dis, 0x33);
	LCD_2IN_WriteData_Byte(dis, 0x33);

	LCD_2IN_Write_Command(dis, 0xB7);
	LCD_2IN_WriteData_Byte(dis, 0x35); 

	LCD_2IN_Write_Command(dis, 0xBB);
	LCD_2IN_WriteData_Byte(dis, 0x1F);

	LCD_2IN_Write_Command(dis, 0xC0);
	LCD_2IN_WriteData_Byte(dis, 0x2C);

	LCD_2IN_Write_Command(dis, 0xC2);
	LCD_2IN_WriteData_Byte(dis, 0x01);

	LCD_2IN_Write_Command(dis, 0xC3);
	LCD_2IN_WriteData_Byte(dis, 0x12);   

	LCD_2IN_Write_Command(dis, 0xC4);
	LCD_2IN_WriteData_Byte(dis, 0x20);

	LCD_2IN_Write_Command(dis, 0xC6);
	LCD_2IN_WriteData_Byte(dis, 0x0F); 

	LCD_2IN_Write_Command(dis, 0xD0);
	LCD_2IN_WriteData_Byte(dis, 0xA4);
	LCD_2IN_WriteData_Byte(dis, 0xA1);

	LCD_2IN_Write_Command(dis, 0xE0);
	LCD_2IN_WriteData_Byte(dis, 0xD0);
	LCD_2IN_WriteData_Byte(dis, 0x08);
	LCD_2IN_WriteData_Byte(dis, 0x11);
	LCD_2IN_WriteData_Byte(dis, 0x08);
	LCD_2IN_WriteData_Byte(dis, 0x0C);
	LCD_2IN_WriteData_Byte(dis, 0x15);
	LCD_2IN_WriteData_Byte(dis, 0x39);
	LCD_2IN_WriteData_Byte(dis, 0x33);
	LCD_2IN_WriteData_Byte(dis, 0x50);
	LCD_2IN_WriteData_Byte(dis, 0x36);
	LCD_2IN_WriteData_Byte(dis, 0x13);
	LCD_2IN_WriteData_Byte(dis, 0x14);
	LCD_2IN_WriteData_Byte(dis, 0x29);
	LCD_2IN_WriteData_Byte(dis, 0x2D);

	LCD_2IN_Write_Command(dis, 0xE1);
	LCD_2IN_WriteData_Byte(dis, 0xD0);
	LCD_2IN_WriteData_Byte(dis, 0x08);
	LCD_2IN_WriteData_Byte(dis, 0x10);
	LCD_2IN_WriteData_Byte(dis, 0x08);
	LCD_2IN_WriteData_Byte(dis, 0x06);
	LCD_2IN_WriteData_Byte(dis, 0x06);
	LCD_2IN_WriteData_Byte(dis, 0x39);
	LCD_2IN_WriteData_Byte(dis, 0x44);
	LCD_2IN_WriteData_Byte(dis, 0x51);
	LCD_2IN_WriteData_Byte(dis, 0x0B);
	LCD_2IN_WriteData_Byte(dis, 0x16);
	LCD_2IN_WriteData_Byte(dis, 0x14);
	LCD_2IN_WriteData_Byte(dis, 0x2F);
	LCD_2IN_WriteData_Byte(dis, 0x31);
	LCD_2IN_Write_Command(dis, 0x21);

	LCD_2IN_Write_Command(dis, 0x11);

	LCD_2IN_Write_Command(dis, 0x29);
}

#[panic_handler]
fn panic(_: &PanicInfo) -> ! {
    // when we panic, do nothing
    loop {}
}

const CMD_SET_X: u8 = 0x2a;
const CMD_SET_Y: u8 = 0x2b;
const CMD_END: u8 = 0x2c;
const LCD_2IN_WIDTH: usize = 240;
const LCD_2IN_HEIGHT: usize = 320;

fn LCD_2IN_SetWindow<T: WriteOnlyDataCommand >(dis: &mut T, Xstart: u16, Ystart: u16, Xend: u16, Yend: u16) {
    
	LCD_2IN_Write_Command(dis, 0x2a);
	LCD_2IN_WriteData_Byte(dis, (Xstart >>8) as u8);
	LCD_2IN_WriteData_Byte(dis, (Xstart & 0xff) as u8);
	LCD_2IN_WriteData_Byte(dis, ((Xend - 1) >> 8) as u8);
	LCD_2IN_WriteData_Byte(dis, ((Xend - 1) & 0xff) as u8);

	LCD_2IN_Write_Command(dis, 0x2b);
	LCD_2IN_WriteData_Byte(dis, (Ystart >>8) as u8);
	LCD_2IN_WriteData_Byte(dis, (Ystart & 0xff) as u8);
	LCD_2IN_WriteData_Byte(dis, ((Yend - 1) >> 8) as u8);
	LCD_2IN_WriteData_Byte(dis, ((Yend - 1) & 0xff) as u8);

	LCD_2IN_Write_Command(dis, 0x2C);
}

fn LCD_2IN_Clear<T: WriteOnlyDataCommand>(dis: &mut T, color: u16) {
    let mut image = [0u16; LCD_2IN_WIDTH];
    for i in 0..LCD_2IN_WIDTH {
        image[i] = color>>8 | (color&0xFF)<<8;
    }
    LCD_2IN_SetWindow(dis, 0, 0, LCD_2IN_WIDTH as u16, LCD_2IN_HEIGHT as u16);
    for i in 0..LCD_2IN_HEIGHT {
        dis.send_data(DataFormat::U16(&image[..]));
    }
}

fn blit<T: WriteOnlyDataCommand>(dis: &mut T, image: &[u16]) {
    LCD_2IN_SetWindow(dis, 0, 0, LCD_2IN_WIDTH as u16, LCD_2IN_HEIGHT as u16);
    for i in 0..LCD_2IN_HEIGHT {
        dis.send_data(DataFormat::U16(&image[(i*LCD_2IN_HEIGHT)..((i+1)*LCD_2IN_HEIGHT)]));
    }
}

fn set_window<T: WriteOnlyDataCommand >(dis: &mut T, x: u16, w:u16, y: u16, h:u16) {
    dis.send_commands(DataFormat::U8(&[CMD_SET_X]));
    dis.send_commands(DataFormat::U16BE(&mut [x]));
    dis.send_commands(DataFormat::U16BE(&mut [x+w]));
    dis.send_commands(DataFormat::U8(&[CMD_SET_Y]));
    dis.send_commands(DataFormat::U16BE(&mut [y]));
    dis.send_commands(DataFormat::U16BE(&mut [y+h]));
    dis.send_commands(DataFormat::U8(&[CMD_END]));
}

fn write_color<T: WriteOnlyDataCommand >(dis: &mut T, color: u16) {
    dis.send_data(DataFormat::U16BE(&mut [color]));
}

fn LCD_2IN_Write_Command<T: WriteOnlyDataCommand>(dis: &mut T, com: u8) {
    dis.send_commands(DataFormat::U8(&[com]));
}

fn LCD_2IN_WriteData_Byte<T: WriteOnlyDataCommand>(dis: &mut T, com: u8) {
    dis.send_data(DataFormat::U8(&[com]));
}

struct LCD2InchDisplay<T>{
    display: T,
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
    type Error = ();
    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>> {
            for pixel in pixels.into_iter() {
                let x: u16 = pixel.0.x as u16;
                let y: u16 = pixel.0.y as u16;
                LCD_2IN_SetWindow(&mut self.display, x, y, x+1, y+1);
                self.display.send_data(DataFormat::U16BE(&mut [pixel.1.into_storage()]));
            }
            Ok(())
        }
}