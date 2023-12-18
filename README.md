# pio-uart

## Overview
The `pio-uart` crate provides a software UART implementation for the Raspberry Pi RP2040 microcontroller, utilizing its Programmable I/O (PIO) feature. This crate enables serial communication on the RP2040 without using its dedicated UART hardware blocks, allowing for greater flexibility in pin selection and potentially freeing up hardware UARTs for other purposes.

## Features
- **PIO-based UART**: Implements UART communication purely through the RP2040's PIO feature.
- **Flexible Pin Selection**: Any GPIO pin can be used for UART TX and RX, not limited to specific UART pins.
- **Configurable Baud Rate**: Supports setting custom baud rates, subject to the limitations of PIO state machine timing.
- **Tx/Rx Buffering**: Internal buffering for both transmission and reception.
- **Error Handling**: Basic error detection for frame errors.

## Installation
Add `pio-uart` to your Cargo.toml:

```toml
[dependencies]
pio-uart = "0.1.0"
```

## Usage
Basic usage of the `pio-uart` crate involves setting up the PIO UART with desired pins and baud rate, and then using it for reading and writing data.

Example:
```rust
use embedded_io::{Read, Write};
use fugit::RateExtU32;
use pio_uart::PioUart;
use rp2040_hal as hal;
use rp2040_hal::pac;

fn main() {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ, pac.XOSC, pac.CLOCKS,
        pac.PLL_SYS, pac.PLL_USB, &mut pac.RESETS, &mut watchdog,
    ).ok().unwrap();

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

    uart.write(b"Hello, UART over PIO!");
    let mut buffer = [0u8; 10];
    uart.read(&mut buffer);
}
```

## Documentation
For detailed documentation, examples, and API reference, visit [crates.io](https://crates.io/crates/pio-uart).

## License
This crate is licensed under the [BSD-3-Clause license](LICENSE).

## Contribution
Contributions are welcome. Please follow the standard Rust community contribution guidelines.

## Disclaimer
This crate is provided as-is, with no guarantees of functionality or stability. The developers are not responsible for any damage caused by using this crate. 