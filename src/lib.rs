#![no_std]

use rp2040_hal::{
    gpio::{FunctionPio0, FunctionPio1, Pin, PinId, PullNone, PullUp},
    pac,
    pio::{self, PIOBuilder, PIOExt, ShiftDirection, StateMachine, SM1, SM2, SM3},
};

pub struct PioUart<RXID: PinId, TXID: PinId, State> {
    rx: pio::Rx<(pac::PIO1, pio::SM0)>,
    tx: pio::Tx<(pac::PIO0, pio::SM0)>,
    rx_sm: StateMachine<(pac::PIO1, pio::SM0), State>,
    tx_sm: StateMachine<(pac::PIO0, pio::SM0), State>,
    // The following fields are use to restore the original state in `free()`
    _rx_pin: Pin<RXID, FunctionPio1, PullUp>,
    _tx_pin: Pin<TXID, FunctionPio0, PullNone>,
    _pio0: (
        pio::PIO<pac::PIO0>,
        pio::Rx<(pac::PIO0, pio::SM0)>,
        pio::UninitStateMachine<(pac::PIO0, SM1)>,
        pio::UninitStateMachine<(pac::PIO0, SM2)>,
        pio::UninitStateMachine<(pac::PIO0, SM3)>,
    ),
    _pio1: (
        pio::PIO<pac::PIO1>,
        pio::Tx<(pac::PIO1, pio::SM0)>,
        pio::UninitStateMachine<(pac::PIO1, SM1)>,
        pio::UninitStateMachine<(pac::PIO1, SM2)>,
        pio::UninitStateMachine<(pac::PIO1, SM3)>,
    ),
}

impl<RXID: PinId, TXID: PinId> PioUart<RXID, TXID, pio::Stopped> {
    pub fn new(
        pio0: pac::PIO0,
        pio1: pac::PIO1,
        rx_pin: Pin<RXID, FunctionPio1, PullUp>,
        tx_pin: Pin<TXID, FunctionPio0, PullNone>,
        resets: &mut pac::RESETS,
        baud: fugit::HertzU32,
        system_freq: fugit::HertzU32,
    ) -> Self {
        let div = system_freq.to_Hz() as f32 / (8f32 * baud.to_Hz() as f32);
        let rx_id = rx_pin.id().num;
        let tx_id = tx_pin.id().num;

        let program_with_defines = pio_proc::pio_file!("src/uart_tx.pio",);
        let program = program_with_defines.program;
        let (mut pio0, sm0_0, sm0_1, sm0_2, sm0_3) = pio0.split(resets);
        let program = pio0.install(&program).expect("PIO");
        let builder = PIOBuilder::from_program(program);
        let (mut tx_sm, tx_rx, tx) = builder
            .out_pins(tx_id, 1)
            .side_set_pin_base(tx_id)
            .out_shift_direction(ShiftDirection::Right)
            .autopull(false)
            .pull_threshold(32)
            .buffers(pio::Buffers::OnlyTx)
            .build(sm0_0);
        tx_sm.set_pindirs([(tx_id, pio::PinDir::Output)].into_iter());
        tx_sm.set_clock_divisor(div);
        // let tx_sm = sm_tx.start();

        let program_with_defines =
            pio_proc::pio_file!("src/uart_rx.pio", select_program("uart_rx"));
        let program = program_with_defines.program;
        let (mut pio1, sm1_0, sm1_1, sm1_2, sm1_3) = pio1.split(resets);
        let program = pio1.install(&program).expect("PIO");
        let builder = PIOBuilder::from_program(program);
        let (mut rx_sm, rx, rx_tx) = builder
            .in_pin_base(rx_id)
            .jmp_pin(rx_id)
            .in_shift_direction(ShiftDirection::Right)
            .autopush(false)
            .push_threshold(32)
            .buffers(pio::Buffers::OnlyRx)
            .build(sm1_0);
        rx_sm.set_pindirs([(rx_id, pio::PinDir::Input)].into_iter());
        rx_sm.set_clock_divisor(div);
        // let rx_sm = sm_rx.start();
        Self {
            _rx_pin: rx_pin,
            _tx_pin: tx_pin,
            rx,
            tx,
            _pio0: (pio0, tx_rx, sm0_1, sm0_2, sm0_3),
            _pio1: (pio1, rx_tx, sm1_1, sm1_2, sm1_3),
            rx_sm,
            tx_sm,
        }
    }
    #[inline]
    pub fn enable(self) -> PioUart<RXID, TXID, pio::Running> {
        PioUart {
            _rx_pin: self._rx_pin,
            _tx_pin: self._tx_pin,
            _pio0: self._pio0,
            _pio1: self._pio1,
            rx: self.rx,
            tx: self.tx,
            rx_sm: self.rx_sm.start(),
            tx_sm: self.tx_sm.start(),
        }
    }
    pub fn free(
        self,
    ) -> (
        pac::PIO0,
        pac::PIO1,
        Pin<RXID, FunctionPio1, PullUp>,
        Pin<TXID, FunctionPio0, PullNone>,
    ) {
        let sm0_0 = self.tx_sm.uninit(self._pio0.1, self.tx);
        let sm1_0 = self.rx_sm.uninit(self.rx, self._pio1.1);
        let pio0 = self
            ._pio0
            .0
            .free(sm0_0.0, self._pio0.2, self._pio0.3, self._pio0.4);
        let pio1 = self
            ._pio1
            .0
            .free(sm1_0.0, self._pio1.2, self._pio1.3, self._pio1.4);
        (pio0, pio1, self._rx_pin, self._tx_pin)
    }
}
impl<RXID: PinId, TXID: PinId> PioUart<RXID, TXID, pio::Running> {
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
    pub fn write_raw(&mut self, buf: &[u8]) -> Result<(), ()> {
        for b in buf {
            while self.tx.is_full() {
                core::hint::spin_loop()
            }
            self.tx.write(*b as u32);
        }
        Ok(())
    }
    fn flush(&mut self) {
        while !self.tx.is_empty() {
            core::hint::spin_loop()
        }
        //FIXME This was found by trial and error
        cortex_m::asm::delay(500 * 125);
    }
    #[inline]
    pub fn stop(self) -> PioUart<RXID, TXID, pio::Stopped> {
        PioUart {
            _rx_pin: self._rx_pin,
            _tx_pin: self._tx_pin,
            _pio0: self._pio0,
            _pio1: self._pio1,
            rx: self.rx,
            tx: self.tx,
            rx_sm: self.rx_sm.stop(),
            tx_sm: self.tx_sm.stop(),
        }
    }
}

#[derive(core::fmt::Debug, defmt::Format)]
pub struct PioSerialError;

impl embedded_io::Error for PioSerialError {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}
impl<RXID: PinId, TXID: PinId> embedded_io::ErrorType for PioUart<RXID, TXID, pio::Running> {
    type Error = PioSerialError;
}
impl<RXID: PinId, TXID: PinId> embedded_io::Read for PioUart<RXID, TXID, pio::Running> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.read_raw(buf).map_err(|_| PioSerialError)
    }
}
impl<RXID: PinId, TXID: PinId> embedded_io::Write for PioUart<RXID, TXID, pio::Running> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.write_raw(buf)
            .map(|_| buf.len())
            .map_err(|_| PioSerialError)
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush();
        Ok(())
    }
}
