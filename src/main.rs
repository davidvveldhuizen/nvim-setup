#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

extern crate alloc;

use alloc::{boxed::Box, vec::Vec};
use alloc_cortex_m::CortexMHeap;
use core::mem::MaybeUninit;
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts, peripherals,
    rcc::{ClockSrc, Pll, PllMul, PllPreDiv, PllRDiv, PllSource},
    usart::{self, BufferedUart},
};
use embedded_io_async::Read;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irgs {
    USART3   => usart::BufferedInterruptHandler<peripherals::USART3>;
});

static mut HEAP_SPACE: [MaybeUninit<u8>; 32 * 1024] = [MaybeUninit::uninit(); 32 * 1024];

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    unsafe {
        ALLOCATOR.init(HEAP_SPACE.as_mut_ptr() as usize, HEAP_SPACE.len());
    }

    //configure the clockspeed
    let p = embassy_stm32::init({
        let mut hal_config = embassy_stm32::Config::default();
        hal_config.enable_debug_during_sleep = true;
        hal_config.rcc.hsi = true;
        hal_config.rcc.mux = ClockSrc::PLL1_R;
        hal_config.rcc.pll = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL10,
            divp: None,
            divq: None,
            divr: Some(PllRDiv::DIV2),
        });
        hal_config
    });

    let mut buffer: Vec<u8> = Vec::new();

    //configure the uart
    let mut conf = usart::Config::default();
    conf.baudrate = 115200;
    let mut tx_buffer = [0u8; 128];
    let mut rx_buffer = [0u8];
    let mut uart = BufferedUart::new(
        p.USART3,
        Irgs,
        p.PC11,
        p.PC10,
        &mut tx_buffer,
        &mut rx_buffer,
        conf,
    )
    .unwrap();

    loop {
        let line = next_line(&mut uart, &mut buffer).await;
        info!("line: {:?}", core::str::from_utf8(&line).unwrap());
    }
}

async fn next_line<U>(rx: &mut U, buffer: &mut Vec<u8>) -> Vec<u8>
where
    U: Read,
    U::Error: defmt::Format,
{
    let mut line_buffer = [0u8; 1024]; //buffer to store a single line

    loop {
        if let Some(pos) = buffer.iter().position(|l| l == &b'\n') {
            // if the line buffer contains a full line the buffer gets split and the function
            // returns te line, the part without the line gets put back in the buffer
            let mut line = core::mem::take(buffer);
            *buffer = line.split_off(pos + 1);
            return line;
        }

        //prevent overflow of the buffer
        if buffer.len() > 1024 {
            buffer.clear();
        }

        match rx.read(&mut line_buffer).await {
            Ok(len) => buffer.extend_from_slice(&line_buffer[..len]),
            Err(e) => defmt::info!("failed to read: {}", e),
        }
    }
}
