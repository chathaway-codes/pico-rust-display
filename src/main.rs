// Turn off std and main; they assume some features are available
// which aren't (i.e., a heap).
#![no_std]
#![no_main]

// We need to define our own panic handler
use core::panic::PanicInfo;
use embedded_hal::digital::v2::InputPin;

use embedded_graphics::primitives::{PrimitiveStyleBuilder, Rectangle};
use embedded_graphics::{image::Image, pixelcolor::Rgb565, prelude::*};
use tinybmp::Bmp;
use display_interface_spi::SPIInterface;

use st7789::ST7789;

// peripheral access crate
//use rp_pico::hal::{self, pac};

use embedded_hal::spi::MODE_0;
use fugit::RateExtU32;
use rp2040_hal::{gpio::Pins, pac, sio::Sio, spi::Spi};

use rp_pico::{
    self,
    hal::{self, prelude::*},
};

use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, MonoTextStyle},
    text::Text,
};

mod display;

use display::LCD2InchDisplay;

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

    let spi = Spi::<_, _, 8>::new(pac.SPI0).init(
        &mut pac.RESETS,
        RateExtU32::Hz(525_000_000u32),
        RateExtU32::Hz(525_000_000u32),
        &MODE_0,
    );
    let dc = pins.gpio6.into_push_pull_output();
    let cs = pins.gpio5.into_push_pull_output();
    let reset = pins.gpio16.into_push_pull_output();
    let button_pin = pins.gpio17.into_pull_down_input();
    let idk_pin = pins.gpio27.into_push_pull_output();

    //let mut display = LCD2InchDisplay::new(spi, dc, cs, reset, delay).unwrap();
    let mut spi = SPIInterface::new(spi, dc, cs);
    let mut display = ST7789::new(spi, Some(reset), Some(idk_pin), 240,320);
    display.init(&mut delay).unwrap();

    let bmp_data = include_bytes!("../image.bmp");
    let bmp: Bmp<Rgb565> = Bmp::from_slice(bmp_data).unwrap();
    let image = Image::new(&bmp, Point::new(0, 0));
    image.draw(&mut display).unwrap();
    Text::new(
        "Tacos are good",
        Point::new(15, 15),
        MonoTextStyle::new(&FONT_10X20, Rgb565::new(0xFF, 0xFF, 0xFF)),
    )
    .draw(&mut display)
    .unwrap();

    Rectangle::new(Point::new(16, 24), Size::new(200, 200))
        .into_styled(
            PrimitiveStyleBuilder::new()
                .stroke_width(2)
                .stroke_color(Rgb565::RED)
                .fill_color(Rgb565::CYAN)
                .build(),
        )
        .draw(&mut display)
        .unwrap();

    let mut second_message = false;

    loop {
        if button_pin.is_high().unwrap() && !second_message {
            Text::new(
                "Do you like tacos?",
                Point::new(15, 200),
                MonoTextStyle::new(&FONT_10X20, Rgb565::new(0xFF, 0xFF, 0xFF)),
            )
            .draw(&mut display)
            .unwrap();
            second_message = true;
        }
    }
}

#[panic_handler]
fn panic(_: &PanicInfo) -> ! {
    // when we panic, do nothing
    loop {}
}
