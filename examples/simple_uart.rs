#![no_std]
#![no_main]

use embedded_io::{Read, Write};
use fugit::RateExtU32;
use pio_uart::PioUart;
use rp2040_hal::{self as hal, pac};
use rp_pico::entry;

use panic_halt as _;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let _core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let _clocks = hal::clocks::init_clocks_and_plls(
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

    let sio = hal::Sio::new(pac.SIO);
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Initialize software UART
    let mut uart = PioUart::new(
        pac.PIO0,
        pins.gpio16.reconfigure(),
        pins.gpio17.reconfigure(),
        &mut pac.RESETS,
        19200.Hz(),
        125.MHz(),
    )
    .enable();

    loop {
        uart.write(b"Hello, UART over PIO!").ok();
        let mut buffer = [0u8; 10];
        uart.read(&mut buffer).ok();
    }
}
