//! The starter code slowly blinks the LED, sets up
//! USB logging, and creates a UART driver using pins
//! 14 and 15. The UART baud rate is [`UART_BAUD`].
//!
//! Despite targeting the Teensy 4.0, this starter code
//! also works on the Teensy 4.1.

#![no_std]
#![no_main]

use nb;
use core::time::Duration;
use embedded_hal::blocking::spi::write;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::prelude::{_embedded_hal_blocking_delay_DelayMs, _embedded_hal_blocking_spi_Write, _embedded_hal_timer_CountDown};
use embedded_hal::spi;
use embedded_hal::spi::FullDuplex;
use smart_leds::{brightness, hsv::RGB8, SmartLedsWrite};
use teensy4_bsp::board::T40Resources;
use teensy4_bsp::board;
use teensy4_bsp::hal::gpt::ClockSource;
use teensy4_bsp::hal::timer;
use teensy4_bsp::pins::{t40, tmm};
use teensy4_panic as _;
use ws2812_spi::Ws2812;
use fugit::{ExtU32, Megahertz, NanosDuration};
use imxrt_hal;
use imxrt_ral;
use imxrt_hal::ccm::{self, clock_gate, perclk_clk};
use imxrt_hal::dma::channel::Channel;
use imxrt_hal::lpspi::{Lpspi, LpspiError};
use imxrt_hal as hal;
use nb::block;
use teensy4_panic::sos;

const DELAY_MS: u32 = 500;

#[teensy4_bsp::rt::entry]
fn main() -> ! {
    // These are peripheral instances. Let the board configure these for us.
    // This function can only be called once!
    let instances = board::instances();

    // Driver resources that are configured by the board. For more information,
    // see the `board` documentation.
    let board::Resources {
        mut gpio1,
        mut gpio2,
        mut gpio3,
        mut gpio4,
        mut lpspi4,
        // `pins` has objects that represent the physical pins. The object
        // for pin 13 is `p13`.
        pins,
        // This is a hardware timer. We'll use it for blocking delays.
        mut gpt1,
        mut gpt2,
        ..
    } = board::t40(instances);

    // Configures the GPT1 timer to run at GPT1_FREQUENCY. See the
    // constants below for more information.
    gpt2.disable();
    gpt2.set_divider(GPT1_DIVIDER);
    gpt2.set_clock_source(GPT1_CLOCK_SOURCE);
    let mut delay = timer::Blocking::<_, GPT1_FREQUENCY>::from_gpt(gpt2);

    // let mut led = board::led(&mut gpio2, pins.p13);
    let mut pin = gpio4.output(pins.p2);

    const NUM_LEDS: usize = 10;
    let mut data = [RGB8::default(); NUM_LEDS];
    for i in 0..NUM_LEDS {
        data[i] = RGB8::new(0, 255, 0);
    }

    let mut spi: board::Lpspi4 = board::lpspi(
        lpspi4,
        board::LpspiPins {
            pcs0: pins.p10,
            sck: pins.p13,
            sdo: pins.p11,
            sdi: pins.p12,
        },
        3_000_000,
    );

    // let mut channels = hal::dma::channels(
    //     hal::ral::dma0::DMA0::take().unwrap(),
    //     hal::ral::dmamux::DMAMUX::take().unwrap(),
    // );
    //
    // let mut tx_channel = channels[8].take().unwrap();
    // let mut rx_channel = channels[9].take().unwrap();
    // tx_channel.set_interrupt_on_completion(true);
    // rx_channel.set_interrupt_on_completion(true);
    //
    // let duplex = spi.dma_full_duplex(&mut rx_channel, &mut tx_channel, &mut buffer).unwrap();
    //

    let mut ws = Ws2812::new(NeuSpi { spi: spi });

    loop {
        // led.set_high();
        // delay.delay_ms(DELAY_MS);
        // led.set_low();
        // delay.delay_ms(DELAY_MS);
        // let res = pin.set_high();
        pin.set_high();
        let res = ws.write(data.into_iter());
        match res {
            Ok(value) => {
                pin.toggle();
            },
            Err(error) => {
                pin.set_high();
            },
        }
        delay.delay_ms(DELAY_MS);
        pin.set_high();
    }
}

// We're responsible for configuring our timers.
// This example uses PERCLK_CLK as the GPT1 clock source,
// and it configures a 1 KHz GPT1 frequency by computing a
// GPT1 divider.

/// The intended GPT1 frequency (Hz).
const GPT1_FREQUENCY: u32 = 3_000;
/// Given this clock source...
const GPT1_CLOCK_SOURCE: ClockSource = ClockSource::HighFrequencyReferenceClock;
/// ... the root clock is PERCLK_CLK. To configure a GPT1 frequency,
/// we need a divider of...
const GPT1_DIVIDER: u32 = board::PERCLK_FREQUENCY / GPT1_FREQUENCY;

struct NeuSpi {
    pub spi: board::Lpspi4,
}

impl FullDuplex<u8> for NeuSpi {

    type Error = ();

    fn read(&mut self) -> nb::Result<u8, ()> {
        let res = self.spi.read_data();
        match res {
            None => {
                return nb::Result::Err(nb::Error::WouldBlock);
            }
            Some(value) => {
                return nb::Result::Ok(value as u8);
            }
        }
    }

    fn send(&mut self, word: u8) -> nb::Result<(), ()> {
        let res = self.spi.write(&[word]);
        match res {
            Ok(_) => {
                return nb::Result::Ok(())
            }
            Err(_) => {
                return nb::Result::Err(nb::Error::WouldBlock);
            }
        }
    }
}