use atomic_polyfill::{compiler_fence, Ordering};
use core::future::Future;
use core::marker::PhantomData;
use core::pin::Pin;
use core::task::Context;
use core::task::Poll;
use embassy::util::{Unborrow, WakerRegistration};
use embassy_hal_common::peripheral::{PeripheralMutex, PeripheralState, StateStorage};
use embassy_hal_common::ring_buffer::RingBuffer;
use embassy_hal_common::unborrow;
use futures::TryFutureExt;

use super::*;
use crate::dma::NoDma;
use crate::pac::usart::{regs, vals};

pub struct Uart<'d, T: Instance, TxDma = NoDma, RxDma = NoDma> {
    inner: T,
    phantom: PhantomData<&'d mut T>,
    tx_dma: TxDma,
    #[allow(dead_code)]
    rx_dma: RxDma,
}

impl<'d, T: Instance, TxDma, RxDma> Uart<'d, T, TxDma, RxDma> {
    pub fn new(
        inner: impl Unborrow<Target = T>,
        rx: impl Unborrow<Target = impl RxPin<T>>,
        tx: impl Unborrow<Target = impl TxPin<T>>,
        tx_dma: impl Unborrow<Target = TxDma>,
        rx_dma: impl Unborrow<Target = RxDma>,
        config: Config,
    ) -> Self {
        unborrow!(inner, rx, tx, tx_dma, rx_dma);

        T::enable();
        let pclk_freq = T::frequency();

        // TODO: better calculation, including error checking and OVER8 if possible.
        let div = (pclk_freq.0 + (config.baudrate / 2)) / config.baudrate;

        let r = inner.regs();

        unsafe {
            rx.set_as_af(rx.af_num());
            tx.set_as_af(tx.af_num());

            r.brr().write_value(regs::Brr(div));
            r.cr1().write(|w| {
                w.set_ue(true);
                w.set_te(true);
                w.set_re(true);
                w.set_m(vals::M::M8);
                w.set_pce(config.parity != Parity::ParityNone);
                w.set_ps(match config.parity {
                    Parity::ParityOdd => vals::Ps::ODD,
                    Parity::ParityEven => vals::Ps::EVEN,
                    _ => vals::Ps::EVEN,
                });
            });
            r.cr2().write(|_w| {});
            r.cr3().write(|_w| {});
        }

        Self {
            inner,
            phantom: PhantomData,
            tx_dma,
            rx_dma,
        }
    }

    async fn write_dma(&mut self, buffer: &[u8]) -> Result<(), Error>
    where
        TxDma: crate::usart::TxDma<T>,
    {
        let ch = &mut self.tx_dma;
        unsafe {
            self.inner.regs().cr3().modify(|reg| {
                reg.set_dmat(true);
            });
        }
        let r = self.inner.regs();
        let dst = r.dr().ptr() as *mut u8;
        ch.write(ch.request(), buffer, dst).await;
        Ok(())
    }

    async fn read_dma(&mut self, buffer: &mut [u8]) -> Result<(), Error>
    where
        RxDma: crate::usart::RxDma<T>,
    {
        let ch = &mut self.rx_dma;
        unsafe {
            self.inner.regs().cr3().modify(|reg| {
                reg.set_dmar(true);
            });
        }
        let r = self.inner.regs();
        let src = r.dr().ptr() as *mut u8;
        ch.read(ch.request(), src, buffer).await;
        Ok(())
    }

    pub fn read_blocking(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        unsafe {
            let r = self.inner.regs();
            for b in buffer {
                loop {
                    let sr = r.sr().read();
                    if sr.pe() {
                        r.dr().read();
                        return Err(Error::Parity);
                    } else if sr.fe() {
                        r.dr().read();
                        return Err(Error::Framing);
                    } else if sr.ne() {
                        r.dr().read();
                        return Err(Error::Noise);
                    } else if sr.ore() {
                        r.dr().read();
                        return Err(Error::Overrun);
                    } else if sr.rxne() {
                        break;
                    }
                }
                *b = r.dr().read().0 as u8;
            }
        }
        Ok(())
    }
}

impl<'d, T: Instance, RxDma> embedded_hal::blocking::serial::Write<u8>
    for Uart<'d, T, NoDma, RxDma>
{
    type Error = Error;
    fn bwrite_all(&mut self, buffer: &[u8]) -> Result<(), Self::Error> {
        unsafe {
            let r = self.inner.regs();
            for &b in buffer {
                while !r.sr().read().txe() {}
                r.dr().write_value(regs::Dr(b as u32))
            }
        }
        Ok(())
    }
    fn bflush(&mut self) -> Result<(), Self::Error> {
        unsafe {
            let r = self.inner.regs();
            while !r.sr().read().tc() {}
        }
        Ok(())
    }
}

// rustfmt::skip because intellij removes the 'where' claus on the associated type.
#[rustfmt::skip]
impl<'d, T: Instance, TxDma, RxDma> embassy_traits::uart::Write for Uart<'d, T, TxDma, RxDma>
    where TxDma: crate::usart::TxDma<T>
{
    type WriteFuture<'a> where Self: 'a = impl Future<Output = Result<(), embassy_traits::uart::Error>> + 'a;

    fn write<'a>(&'a mut self, buf: &'a [u8]) -> Self::WriteFuture<'a> {
        self.write_dma(buf).map_err(|_| embassy_traits::uart::Error::Other)
    }
}

// rustfmt::skip because intellij removes the 'where' claus on the associated type.
#[rustfmt::skip]
impl<'d, T: Instance, TxDma, RxDma> embassy_traits::uart::Read for Uart<'d, T, TxDma, RxDma>
    where RxDma: crate::usart::RxDma<T>
{
    type ReadFuture<'a> where Self: 'a = impl Future<Output = Result<(), embassy_traits::uart::Error>> + 'a;

    fn read<'a>(&'a mut self, buf: &'a mut [u8]) -> Self::ReadFuture<'a> {
        self.read_dma(buf).map_err(|_| embassy_traits::uart::Error::Other)
    }
}

pub struct State<'d, T: Instance>(StateStorage<StateInner<'d, T>>);
impl<'d, T: Instance> State<'d, T> {
    pub fn new() -> Self {
        Self(StateStorage::new())
    }
}

pub struct StateInner<'d, T: Instance> {
    uart: Uart<'d, T, NoDma, NoDma>,
    phantom: PhantomData<&'d mut T>,

    rx_waker: WakerRegistration,
    rx: RingBuffer<'d>,

    tx_waker: WakerRegistration,
    tx: RingBuffer<'d>,
}

unsafe impl<'d, T: Instance> Send for StateInner<'d, T> {}
unsafe impl<'d, T: Instance> Sync for StateInner<'d, T> {}

pub struct BufferedUart<'d, T: Instance> {
    inner: PeripheralMutex<'d, StateInner<'d, T>>,
}

impl<'d, T: Instance> Unpin for BufferedUart<'d, T> {}

impl<'d, T: Instance> BufferedUart<'d, T> {
    pub unsafe fn new(
        state: &'d mut State<'d, T>,
        uart: Uart<'d, T, NoDma, NoDma>,
        irq: impl Unborrow<Target = T::Interrupt> + 'd,
        tx_buffer: &'d mut [u8],
        rx_buffer: &'d mut [u8],
    ) -> BufferedUart<'d, T> {
        unborrow!(irq);

        let r = uart.inner.regs();
        r.cr1().modify(|w| {
            w.set_rxneie(true);
            w.set_idleie(true);
        });

        Self {
            inner: PeripheralMutex::new_unchecked(irq, &mut state.0, move || StateInner {
                uart,
                phantom: PhantomData,
                tx: RingBuffer::new(tx_buffer),
                tx_waker: WakerRegistration::new(),

                rx: RingBuffer::new(rx_buffer),
                rx_waker: WakerRegistration::new(),
            }),
        }
    }
}

impl<'d, T: Instance> StateInner<'d, T>
where
    Self: 'd,
{
    fn on_rx(&mut self) {
        let r = self.uart.inner.regs();
        unsafe {
            let sr = r.sr().read();
            let dr = r.dr().read();

            if sr.rxne() {
                if sr.pe() {
                    info!("rx parity error");
                }
                if sr.fe() {
                    info!("rx framing error");
                }
                if sr.ne() {
                    info!("rx noise error");
                }
                if sr.ore() {
                    warn!("rx overrun error");
                }

                trace!("rxne");
                let buf = self.rx.push_buf();
                if buf.is_empty() {
                    warn!("rx buffer overrun");
                } else {
                    buf[0] = dr.0 as u8;
                    self.rx.push(1);
                }

                if self.rx.is_full() {
                    trace!("rxbuf full");
                    self.rx_waker.wake();

                    // Disable interrupt until we have space to receive again
                    r.cr1().modify(|w| {
                        w.set_rxneie(false);
                    });
                    r.cr1.write()
                }
            }

            if sr.idle() {
                trace!("idle");
                r.dr().read(); // clear isr
                self.rx_waker.wake();
            };
        }
    }

    fn on_tx(&mut self) {
        let r = self.uart.inner.regs();
        unsafe {
            if r.sr().read().txe() && r.cr1().read().txeie() {
                trace!("txe");
                let buf = self.tx.pop_buf();
                if !buf.is_empty() {
                    trace!("sending next byte");
                    r.dr().write_value(regs::Dr(buf[0].into()));
                    self.tx.pop(1);
                    self.tx_waker.wake();
                } else {
                    trace!("tx done");
                    // Disable interrupt until we have something to transmit again
                    r.cr1().modify(|w| {
                        w.set_txeie(false);
                    });
                }
            }
        }
    }
}

impl<'d, T: Instance> PeripheralState for StateInner<'d, T>
where
    Self: 'd,
{
    type Interrupt = T::Interrupt;
    fn on_interrupt(&mut self) {
        trace!("usart irq");
        self.on_rx();
        self.on_tx();
    }
}

impl<'d, T: Instance> embassy::io::AsyncBufRead for BufferedUart<'d, T> {
    fn poll_fill_buf(
        mut self: Pin<&mut Self>,
        cx: &mut Context<'_>,
    ) -> Poll<Result<&[u8], embassy::io::Error>> {
        trace!("poll_read");
        self.inner.with(|state| {
            compiler_fence(Ordering::SeqCst);

            // We have data ready in buffer? Return it.
            let buf = state.rx.pop_buf();
            if !buf.is_empty() {
                let buf: &[u8] = buf;
                // Safety: buffer lives as long as uart
                let buf: &[u8] = unsafe { core::mem::transmute(buf) };
                return Poll::Ready(Ok(buf));
            }

            state.rx_waker.register(cx.waker());
            Poll::<Result<&[u8], embassy::io::Error>>::Pending
        })
    }

    fn consume(mut self: Pin<&mut Self>, amt: usize) {
        self.inner.with(|state| {
            trace!("consume n={}", amt);
            let was_full = state.rx.is_full();
            state.rx.pop(amt);
            if was_full {
                trace!("rxbuf not full anymore");
                // safety: exclusive access guaranteed the peripheral mutex
                unsafe {
                    state.uart.inner.regs().cr1().modify(|w| w.set_rxneie(true));
                }
            }
        });
        //trace!("pend");
        //self.inner.pend();
    }
}

impl<'d, T: Instance> embassy::io::AsyncWrite for BufferedUart<'d, T> {
    fn poll_write(
        mut self: Pin<&mut Self>,
        cx: &mut Context<'_>,
        buf: &[u8],
    ) -> Poll<Result<usize, embassy::io::Error>> {
        let poll = self.inner.with(|state| {
            if state.tx.is_empty() {
                // safety: exclusive access guaranteed the peripheral mutex
                unsafe {
                    state.uart.inner.regs().cr1().modify(|w| w.set_txeie(true));
                }
            }

            let tx_buf = state.tx.push_buf();
            if tx_buf.is_empty() {
                // No more space left in the buffer
                trace!("poll_write tx buffer full");
                state.tx_waker.register(cx.waker());
                return Poll::Pending;
            }

            let n = core::cmp::min(tx_buf.len(), buf.len());
            trace!("poll_write copy {} bytes", n);
            tx_buf[..n].copy_from_slice(&buf[..n]);
            state.tx.push(n);

            Poll::Ready(Ok(n))
        });
        poll
    }
}
