#![no_std]
#![no_main]
#![feature(trait_alias)]
#![feature(type_alias_impl_trait)]
#![allow(incomplete_features)]

#[path = "../example_common.rs"]
mod example_common;

use example_common::*;

use defmt::panic;
use embassy::executor::Spawner;
use embassy::io::{AsyncBufReadExt, AsyncWriteExt};
use embassy_stm32::dbgmcu::Dbgmcu;
use embassy_stm32::dma::NoDma;
use embassy_stm32::interrupt;
use embassy_stm32::usart::{BufferedUart, Config, State, Uart};
use embassy_stm32::Peripherals;

#[embassy::main]
async fn main(_spawner: Spawner, p: Peripherals) {
    unsafe {
        Dbgmcu::enable_all();
    }

    static mut TX_BUFFER: [u8; 16] = [0; 16];
    static mut RX_BUFFER: [u8; 16] = [0; 16];

    let mut config = Config::default();
    config.baudrate = 19200;

    let usart = Uart::new(p.USART2, p.PA3, p.PA2, NoDma, NoDma, config);
    let mut state = State::new();
    let mut usart = unsafe {
        BufferedUart::new(
            &mut state,
            usart,
            interrupt::take!(USART2),
            &mut TX_BUFFER,
            &mut RX_BUFFER,
        )
    };

    //unwrap!(usart.write_all(b"Hello Embassy World!\r\n").await);
    //info!("wrote Hello, starting echo");

    let mut buf = [0; 8];
    loop {
        //unwrap!(usart.write_all(b"Hello Embassy World!\r\n").await);
        let n = unwrap!(usart.read(&mut buf[..]).await);
        info!("recv n={}", n);
        unwrap!(usart.write_all(&buf[..n]).await);
    }
}
