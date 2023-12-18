#![no_std]
#![no_main]

use embedded_io::{Read, Write};
use fugit::RateExtU32;
use rp2040_hal::{self as hal, pac, pio::PIOExt};
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

    // Split PIO0 to be able to program it
    let (mut pio, sm0, sm1, _sm2, _sm3) = pac.PIO0.split(&mut pac.RESETS);
    // Program RX and TX programs into PIO0
    let mut rx_program = pio_uart::install_rx_program(&mut pio).ok().unwrap();
    let mut tx_program = pio_uart::install_tx_program(&mut pio).ok().unwrap();

    // Initialize RX
    let mut rx = pio_uart::PioUartRx::new(
        pins.gpio16.reconfigure(),
        sm0,
        &mut rx_program,
        19200.Hz(),
        125.MHz(),
    )
    .enable();

    // Initialize TX
    let mut tx = pio_uart::PioUartTx::new(
        pins.gpio17.reconfigure(),
        sm1,
        &mut tx_program,
        19200.Hz(),
        125.MHz(),
    )
    .enable();

    loop {
        tx.write(b"Hello, UART over PIO!").ok();
        let mut buffer = [0u8; 10];
        rx.read(&mut buffer).ok();
    }
}
