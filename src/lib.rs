//! # PioUart Crate
//!
//! This crate provides a UART implementation using the PIO hardware on the RP2040 microcontroller.
//! It's designed to work with the `rp2040_hal` crate and provides a UART interface through the Programmable I/O (PIO) subsystem.
//!
//! ## Features
//! - UART communication using PIO
//! - Flexible pin assignment for RX and TX
//! - Customizable baud rate and system frequency settings
//! - Non-blocking read and write operations
//!
//! ## Usage
//! To use this crate, ensure that you have `rp2040_hal` and `embedded-hal` as dependencies in your `Cargo.toml`.
//! You'll need to configure the PIO and state machines to set up the UART interface.
//!
//! ## Example
//! ```
//! use pio_uart::PioUart;
//! use embedded_io::{Read, Write};
//! use fugit::ExtU32;
//!
//! fn main() {
//!     // Normal system initialization
//!     let mut pac = pac::Peripherals::take().unwrap();
//!     let core = pac::CorePeripherals::take().unwrap();
//!     let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
//!     let clocks = hal::clocks::init_clocks_and_plls(
//!         rp_pico::XOSC_CRYSTAL_FREQ, pac.XOSC, pac.CLOCKS,
//!         pac.PLL_SYS, pac.PLL_USB, &mut pac.RESETS, &mut watchdog,
//!     ).ok().unwrap();
//!     let sio = hal::Sio::new(pac.SIO);
//!     let pins = rp_pico::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);
//!
//!     // Initialize software UART
//!     let mut uart = pio_uart::PioUart::new(
//!             pac.PIO0,
//!             pins.gpio16.reconfigure(),
//!             pins.gpio17.reconfigure(),
//!             &mut pac.RESETS,
//!             19200.Hz(),
//!             125.MHz(),
//!         );
//!
//!     uart.write(b"Hello, UART over PIO!");
//!     let mut buffer = [0u8; 10];
//!     uart.read(&mut buffer);
//! }
//! ```

#![no_std]
#![deny(missing_docs)]

use rp2040_hal::{
    gpio::{FunctionPio0, FunctionPio1, Pin, PinId, PullNone, PullUp},
    pio::{
        self, InstallError, InstalledProgram, PIOBuilder, PIOExt, ShiftDirection, StateMachine,
        UninitStateMachine, ValidStateMachine,
    },
};

/// Install the UART Rx program in a PIO instance
pub fn install_rx_program<PIO: PIOExt>(
    pio: &mut pio::PIO<PIO>,
) -> Result<RxProgram<PIO>, InstallError> {
    let program_with_defines = pio_proc::pio_file!("src/uart_rx.pio", select_program("uart_rx"));
    let program = program_with_defines.program;
    pio.install(&program).map(|program| RxProgram { program })
}
/// Install the UART Tx program in a PIO instance
pub fn install_tx_program<PIO: PIOExt>(
    pio: &mut pio::PIO<PIO>,
) -> Result<TxProgram<PIO>, InstallError> {
    let program_with_defines = pio_proc::pio_file!("src/uart_tx.pio",);
    let program = program_with_defines.program;
    pio.install(&program).map(|program| TxProgram { program })
}

/// Represents a UART interface using the RP2040's PIO hardware.
///
/// # Type Parameters
/// - `RXID`: The PinId for the RX pin.
/// - `TXID`: The PinId for the TX pin.
/// - `PIO`:  The PIO instance, either pac::PIO0 or pac::PIO1.
/// - `State`: The state of the UART interface, either `pio::Stopped` or `pio::Running`.
pub struct PioUart<RXID: PinId, TXID: PinId, PIO: PIOExt + PioPinFunction, State> {
    rx: PioUartRx<RXID, (PIO, pio::SM0), State>,
    tx: PioUartTx<TXID, (PIO, pio::SM1), State>,
    // The following fields are use to restore the original state in `free()`
    _rx_program: RxProgram<PIO>,
    _tx_program: TxProgram<PIO>,
    _pio: pio::PIO<PIO>,
    _sm2: UninitStateMachine<(PIO, pio::SM2)>,
    _sm3: UninitStateMachine<(PIO, pio::SM3)>,
}

/// Represents the Rx part of a UART interface using the RP2040's PIO hardware.
///
/// # Type Parameters
/// - `PinID`: The PinId for the RX pin.
/// - `SM`:  The state machine to use.
/// - `State`: The state of the UART interface, either `pio::Stopped` or `pio::Running`.
pub struct PioUartRx<PinID: PinId, SM: ValidStateMachine, State>
where
    SM::PIO: PioPinFunction,
{
    rx: pio::Rx<(SM::PIO, SM::SM)>,
    sm: StateMachine<(SM::PIO, SM::SM), State>,
    // The following fields are use to restore the original state in `free()`
    _rx_pin: Pin<PinID, <SM::PIO as PioPinFunction>::PinFunction, PullUp>,
    _tx: pio::Tx<(SM::PIO, SM::SM)>,
}
/// Represents the Tx part of a UART interface using the RP2040's PIO hardware.
///
/// # Type Parameters
/// - `PinID`: The PinId for the TX pin.
/// - `SM`:  The state machine to use.
/// - `State`: The state of the UART interface, either `pio::Stopped` or `pio::Running`.
pub struct PioUartTx<PinID: PinId, SM: ValidStateMachine, State>
where
    SM::PIO: PioPinFunction,
{
    tx: pio::Tx<(SM::PIO, SM::SM)>,
    sm: StateMachine<(SM::PIO, SM::SM), State>,
    // The following fields are use to restore the original state in `free()`
    _tx_pin: Pin<PinID, <SM::PIO as PioPinFunction>::PinFunction, PullNone>,
    _rx: pio::Rx<(SM::PIO, SM::SM)>,
}

/// Token of the already installed UART Rx program. To be obtained with [`install_rx_program`].
pub struct RxProgram<PIO: PIOExt> {
    program: InstalledProgram<PIO>,
}
/// Token of the already installed UART Tx program. To be obtained with [`install_tx_program`].
pub struct TxProgram<PIO: PIOExt> {
    program: InstalledProgram<PIO>,
}

mod private {
    pub trait Sealed {}
}
/// Helper trait to link PIO instances to the appropriate pin functions.
#[doc(hidden)]
pub trait PioPinFunction: private::Sealed {
    type PinFunction: rp2040_hal::gpio::Function;
}
impl private::Sealed for pio::PIO<rp2040_hal::pac::PIO0> {}
impl private::Sealed for pio::PIO<rp2040_hal::pac::PIO1> {}
impl PioPinFunction for pio::PIO<rp2040_hal::pac::PIO0> {
    type PinFunction = FunctionPio0;
}
impl PioPinFunction for pio::PIO<rp2040_hal::pac::PIO1> {
    type PinFunction = FunctionPio1;
}

impl<PinID: PinId, SM: ValidStateMachine> PioUartRx<PinID, SM, pio::Stopped>
where
    SM::PIO: PioPinFunction,
{
    /// Create a new [`PioUartRx`] instance.
    /// Requires the [`RxProgram`] to be already installed (see [`install_rx_program`]).
    ///
    /// # Arguments
    /// - `rx_pin`: The RX pin configured with `FunctionPioX` and `PullUp`. Use [`pin.gpioX.reconfigure()`](https://docs.rs/rp2040-hal/latest/rp2040_hal/gpio/struct.Pin.html#method.reconfigure).
    /// - `sm`: A PIO state machine instance.
    /// - `rx_program`: The installed Rx program.
    /// - `baud`: Desired baud rate.
    /// - `system_freq`: System frequency.
    pub fn new(
        rx_pin: Pin<PinID, <SM::PIO as PioPinFunction>::PinFunction, PullUp>,
        rx_sm: UninitStateMachine<(SM::PIO, SM::SM)>,
        rx_program: &mut RxProgram<SM::PIO>,
        baud: fugit::HertzU32,
        system_freq: fugit::HertzU32,
    ) -> Self {
        let div = system_freq.to_Hz() as f32 / (8f32 * baud.to_Hz() as f32);
        let rx_id = rx_pin.id().num;

        let (rx_sm, rx, tx) = Self::build_rx(rx_program, rx_id, rx_sm, div);

        Self {
            rx,
            sm: rx_sm,
            _rx_pin: rx_pin,
            _tx: tx,
        }
    }
    fn build_rx(
        token: &mut RxProgram<SM::PIO>,
        rx_id: u8,
        sm: UninitStateMachine<(SM::PIO, SM::SM)>,
        div: f32,
    ) -> (
        StateMachine<(SM::PIO, SM::SM), pio::Stopped>,
        pio::Rx<(SM::PIO, SM::SM)>,
        pio::Tx<(SM::PIO, SM::SM)>,
    ) {
        // SAFETY: Program can not be uninstalled, because it can not be accessed
        let program = unsafe { token.program.share() };
        let builder = PIOBuilder::from_program(program);
        let (mut sm, rx, tx) = builder
            .in_pin_base(rx_id)
            .jmp_pin(rx_id)
            .in_shift_direction(ShiftDirection::Right)
            .autopush(false)
            .push_threshold(32)
            .buffers(pio::Buffers::OnlyRx)
            .build(sm);
        sm.set_pindirs([(rx_id, pio::PinDir::Input)].into_iter());
        sm.set_clock_divisor(div);
        (sm, rx, tx)
    }
    /// Enables the UART, transitioning it to the `Running` state.
    ///
    /// # Returns
    /// An instance of `PioUartRx` in the `Running` state.
    #[inline]
    pub fn enable(self) -> PioUartRx<PinID, SM, pio::Running> {
        PioUartRx {
            sm: self.sm.start(),
            rx: self.rx,
            _rx_pin: self._rx_pin,
            _tx: self._tx,
        }
    }
    /// Frees the underlying resources, returning the SM instance and the pin.
    ///
    /// # Returns
    /// A tuple containing the used SM and the RX pin.
    pub fn free(
        self,
    ) -> (
        UninitStateMachine<(SM::PIO, SM::SM)>,
        Pin<PinID, <SM::PIO as PioPinFunction>::PinFunction, PullUp>,
    ) {
        let (rx_sm, _) = self.sm.uninit(self.rx, self._tx);
        (rx_sm, self._rx_pin)
    }
}

impl<PinID: PinId, SM: ValidStateMachine> PioUartTx<PinID, SM, pio::Stopped>
where
    SM::PIO: PioPinFunction,
{
    /// Create a new [`PioUartTx`] instance.
    /// Requires the [`TxProgram`] to be already installed (see [`install_tx_program`]).
    ///
    /// # Arguments
    /// - `tx_pin`: The TX pin configured with `FunctionPioX` and `PullNone`. Use [`pin.gpioX.reconfigure()`](https://docs.rs/rp2040-hal/latest/rp2040_hal/gpio/struct.Pin.html#method.reconfigure).
    /// - `sm`: A PIO state machine instance.
    /// - `tx_program`: The installed Tx program.
    /// - `baud`: Desired baud rate.
    /// - `system_freq`: System frequency.
    pub fn new(
        tx_pin: Pin<PinID, <SM::PIO as PioPinFunction>::PinFunction, PullNone>,
        sm: UninitStateMachine<(SM::PIO, SM::SM)>,
        tx_program: &mut TxProgram<SM::PIO>,
        baud: fugit::HertzU32,
        system_freq: fugit::HertzU32,
    ) -> Self {
        let div = system_freq.to_Hz() as f32 / (8f32 * baud.to_Hz() as f32);
        let tx_id = tx_pin.id().num;

        let (tx_sm, rx, tx) = Self::build_tx(tx_program, tx_id, sm, div);

        Self {
            tx,
            sm: tx_sm,
            _tx_pin: tx_pin,
            _rx: rx,
        }
    }
    fn build_tx(
        token: &mut TxProgram<SM::PIO>,
        tx_id: u8,
        sm: UninitStateMachine<(SM::PIO, SM::SM)>,
        div: f32,
    ) -> (
        StateMachine<(SM::PIO, SM::SM), pio::Stopped>,
        pio::Rx<(SM::PIO, SM::SM)>,
        pio::Tx<(SM::PIO, SM::SM)>,
    ) {
        // SAFETY: Program can not be uninstalled, because it can not be accessed
        let program = unsafe { token.program.share() };
        let builder = PIOBuilder::from_program(program);
        let (mut sm, rx, tx) = builder
            .out_shift_direction(ShiftDirection::Right)
            .autopull(false)
            .pull_threshold(32)
            .buffers(pio::Buffers::OnlyTx)
            .out_pins(tx_id, 1)
            .side_set_pin_base(tx_id)
            .build(sm);
        sm.set_pindirs([(tx_id, pio::PinDir::Output)].into_iter());
        sm.set_clock_divisor(div);
        (sm, rx, tx)
    }
    /// Enables the UART, transitioning it to the `Running` state.
    ///
    /// # Returns
    /// An instance of `PioUartRx` in the `Running` state.
    #[inline]
    pub fn enable(self) -> PioUartTx<PinID, SM, pio::Running> {
        PioUartTx {
            sm: self.sm.start(),
            tx: self.tx,
            _tx_pin: self._tx_pin,
            _rx: self._rx,
        }
    }
    /// Frees the underlying resources, returning the SM instance and the pin.
    ///
    /// # Returns
    /// A tuple containing the used SM and the TX pin.
    pub fn free(
        self,
    ) -> (
        UninitStateMachine<(SM::PIO, SM::SM)>,
        Pin<PinID, <SM::PIO as PioPinFunction>::PinFunction, PullNone>,
    ) {
        let (tx_sm, _) = self.sm.uninit(self._rx, self.tx);
        (tx_sm, self._tx_pin)
    }
}

impl<RXID: PinId, TXID: PinId, PIO: PIOExt + PioPinFunction>
    PioUart<RXID, TXID, PIO, pio::Stopped>
{
    /// Create a new [`PioUart`] instance.
    /// This method consumes the PIO instance and does not allow to use the other 2 state machines.
    /// If more control is required, use [`PioUartRx`] and [`PioUartTx`] individually.
    ///
    /// # Arguments
    /// - `pio`: A PIO instance from the RP2040, either pac::PIO0 or pac::PIO1.
    /// - `rx_pin`: The RX pin configured with `FunctionPioX` and `PullUp`. Use [`pin.gpioX.reconfigure()`](https://docs.rs/rp2040-hal/latest/rp2040_hal/gpio/struct.Pin.html#method.reconfigure).
    /// - `tx_pin`: The TX pin configured with `FunctionPioX` and `PullNone`. Use [`pin.gpioX.reconfigure()`](https://docs.rs/rp2040-hal/latest/rp2040_hal/gpio/struct.Pin.html#method.reconfigure).
    /// - `resets`: A mutable reference to the RP2040 resets.
    /// - `baud`: Desired baud rate.
    /// - `system_freq`: System frequency.
    pub fn new(
        pio: PIO,
        rx_pin: Pin<RXID, <PIO as PioPinFunction>::PinFunction, PullUp>,
        tx_pin: Pin<TXID, <PIO as PioPinFunction>::PinFunction, PullNone>,
        resets: &mut rp2040_hal::pac::RESETS,
        baud: fugit::HertzU32,
        system_freq: fugit::HertzU32,
    ) -> Self {
        let (mut pio, sm0, sm1, sm2, sm3) = pio.split(resets);
        let mut rx_program = install_rx_program(&mut pio).ok().unwrap(); // Should never fail, because no program was loaded yet
        let mut tx_program = install_tx_program(&mut pio).ok().unwrap(); // Should never fail, because no program was loaded yet
        let rx = PioUartRx::new(rx_pin, sm0, &mut rx_program, baud, system_freq);
        let tx = PioUartTx::new(tx_pin, sm1, &mut tx_program, baud, system_freq);
        Self {
            rx,
            tx,
            _rx_program: rx_program,
            _tx_program: tx_program,
            _pio: pio,
            _sm2: sm2,
            _sm3: sm3,
        }
    }

    /// Enables the UART, transitioning it to the `Running` state.
    ///
    /// # Returns
    /// An instance of `PioUart` in the `Running` state.
    #[inline]
    pub fn enable(self) -> PioUart<RXID, TXID, PIO, pio::Running> {
        PioUart {
            rx: self.rx.enable(),
            tx: self.tx.enable(),
            _rx_program: self._rx_program,
            _tx_program: self._tx_program,
            _pio: self._pio,
            _sm2: self._sm2,
            _sm3: self._sm3,
        }
    }
    /// Frees the underlying resources, returning the PIO instance and pins.
    /// Also uninstalls the UART programs.
    ///
    /// # Returns
    /// A tuple containing the PIO, RX pin, and TX pin.
    pub fn free(
        mut self,
    ) -> (
        PIO,
        Pin<RXID, <PIO as PioPinFunction>::PinFunction, PullUp>,
        Pin<TXID, <PIO as PioPinFunction>::PinFunction, PullNone>,
    ) {
        let (tx_sm, tx_pin) = self.tx.free();
        let (rx_sm, rx_pin) = self.rx.free();
        self._pio.uninstall(self._rx_program.program);
        self._pio.uninstall(self._tx_program.program);
        let pio = self._pio.free(rx_sm, tx_sm, self._sm2, self._sm3);
        (pio, rx_pin, tx_pin)
    }
}

impl<PinID: PinId, SM: ValidStateMachine> PioUartRx<PinID, SM, pio::Running>
where
    SM::PIO: PioPinFunction,
{
    /// Reads raw data into a buffer.
    ///
    /// # Arguments
    /// - `buf`: A mutable slice of u8 to store the read data.
    ///
    /// # Returns
    /// `Ok(usize)`: Number of bytes read.
    /// `Err(())`: If an error occurs.
    pub fn read_raw(&mut self, mut buf: &mut [u8]) -> Result<usize, ()> {
        let buf_len = buf.len();
        while let Some(b) = self.rx.read() {
            buf[0] = (b >> 24) as u8;
            buf = &mut buf[1..];
            if buf.len() == 0 {
                break;
            }
        }
        Ok(buf_len - buf.len())
    }
    /// Stops the UART, transitioning it back to the `Stopped` state.
    ///
    /// # Returns
    /// An instance of `PioUartRx` in the `Stopped` state.
    #[inline]
    pub fn stop(self) -> PioUartRx<PinID, SM, pio::Stopped> {
        PioUartRx {
            sm: self.sm.stop(),
            rx: self.rx,
            _rx_pin: self._rx_pin,
            _tx: self._tx,
        }
    }
}
impl<PinID: PinId, SM: ValidStateMachine> PioUartTx<PinID, SM, pio::Running>
where
    SM::PIO: PioPinFunction,
{
    /// Writes raw data from a buffer.
    ///
    /// # Arguments
    /// - `buf`: A slice of u8 containing the data to write.
    ///
    /// # Returns
    /// `Ok(())`: On success.
    /// `Err(())`: If an error occurs.
    pub fn write_raw(&mut self, buf: &[u8]) -> Result<(), ()> {
        for b in buf {
            while self.tx.is_full() {
                core::hint::spin_loop()
            }
            self.tx.write(*b as u32);
        }
        Ok(())
    }
    /// Flushes the UART transmit buffer.
    fn flush(&mut self) {
        while !self.tx.is_empty() {
            core::hint::spin_loop()
        }
        //FIXME This was found by trial and error
        cortex_m::asm::delay(500 * 125);
    }
    /// Stops the UART, transitioning it back to the `Stopped` state.
    ///
    /// # Returns
    /// An instance of `PioUartTx` in the `Stopped` state.
    #[inline]
    pub fn stop(self) -> PioUartTx<PinID, SM, pio::Stopped> {
        PioUartTx {
            sm: self.sm.stop(),
            tx: self.tx,
            _tx_pin: self._tx_pin,
            _rx: self._rx,
        }
    }
}

/// Represents errors that can occur in the PIO UART.
#[derive(core::fmt::Debug, defmt::Format)]
#[non_exhaustive]
pub enum PioSerialError {
    /// General IO error
    IO,
}

impl embedded_io::Error for PioSerialError {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}
impl<PinID: PinId, SM: ValidStateMachine> embedded_io::ErrorType
    for PioUartRx<PinID, SM, pio::Running>
where
    SM::PIO: PioPinFunction,
{
    type Error = PioSerialError;
}
impl<PinID: PinId, SM: ValidStateMachine> embedded_io::ErrorType
    for PioUartTx<PinID, SM, pio::Running>
where
    SM::PIO: PioPinFunction,
{
    type Error = PioSerialError;
}
impl<RXID: PinId, TXID: PinId, PIO: PIOExt + PioPinFunction> embedded_io::ErrorType
    for PioUart<RXID, TXID, PIO, pio::Running>
{
    type Error = PioSerialError;
}
impl<RXID: PinId, RXSM: ValidStateMachine> embedded_io::Read for PioUartRx<RXID, RXSM, pio::Running>
where
    RXSM::PIO: PioPinFunction,
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.read_raw(buf).map_err(|_| PioSerialError::IO)
    }
}
impl<TXID: PinId, TXSM: ValidStateMachine> embedded_io::Write
    for PioUartTx<TXID, TXSM, pio::Running>
where
    TXSM::PIO: PioPinFunction,
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.write_raw(buf)
            .map(|_| buf.len())
            .map_err(|_| PioSerialError::IO)
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush();
        Ok(())
    }
}

impl<RXID: PinId, TXID: PinId, PIO: PIOExt + PioPinFunction> embedded_io::Read
    for PioUart<RXID, TXID, PIO, pio::Running>
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.rx.read(buf)
    }
}
impl<RXID: PinId, TXID: PinId, PIO: PIOExt + PioPinFunction> embedded_io::Write
    for PioUart<RXID, TXID, PIO, pio::Running>
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.tx.write(buf)
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        embedded_io::Write::flush(&mut self.tx)
    }
}
