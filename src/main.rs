//! The starter code slowly blinks the LED, sets up
//! USB logging, and creates a UART driver using pins
//! 14 and 15. The UART baud rate is [`UART_BAUD`].
//!
//! Despite targeting the Teensy 4.0, this starter code
//! also works on the Teensy 4.1.

#![no_std]
#![no_main]

use core::time::Duration;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::prelude::{_embedded_hal_blocking_delay_DelayMs, _embedded_hal_timer_CountDown};
use smart_leds::{brightness, hsv::RGB8, SmartLedsWrite};
use teensy4_bsp::board::T40Resources;
use teensy4_bsp::board;
use teensy4_bsp::hal::gpt::ClockSource;
use teensy4_bsp::hal::timer;
use teensy4_bsp::pins::{t40, tmm};
use teensy4_panic as _;
use ws2812_timer_delay::Ws2812;
use fugit::{ExtU32, Megahertz, NanosDuration};
use imxrt_hal;
use imxrt_ral;
use imxrt_hal::ccm::{self, clock_gate, perclk_clk};

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
        // `pins` has objects that represent the physical pins. The object
        // for pin 13 is `p13`.
        pins,
        // This is a hardware timer. We'll use it for blocking delays.
        mut gpt1,
        mut gpt2,
        ..
    } = board::t40(instances);

    let mut led = board::led(&mut gpio2, pins.p13);

    // Configures the GPT1 timer to run at GPT1_FREQUENCY. See the
    // constants below for more information.
    gpt2.disable();
    gpt2.set_divider(GPT1_DIVIDER);
    gpt2.set_clock_source(GPT1_CLOCK_SOURCE);
    let mut delay = timer::Blocking::<_, GPT1_FREQUENCY>::from_gpt(gpt2);

    let mut ccm = unsafe { imxrt_ral::ccm::CCM::instance() };

// Before touching the PERCLK clock roots, turn off all downstream clock gates.
    clock_gate::PERCLK_CLOCK_GATES.iter().for_each(|loc| loc.set(&mut ccm, clock_gate::OFF));

    // Configure PERCLK to match this frequency:
    const PERCLK_CLK_FREQUENCY_HZ: u32 = ccm::XTAL_OSCILLATOR_HZ / PERCLK_CLK_DIVIDER;
    const PERCLK_CLK_DIVIDER: u32 = 24;
    perclk_clk::set_selection(&mut ccm, perclk_clk::Selection::Oscillator);
    perclk_clk::set_divider(&mut ccm, PERCLK_CLK_DIVIDER);

// Enable the clock gate for our GPT.
    clock_gate::gpt_bus::<1>().set(&mut ccm, clock_gate::ON);
    clock_gate::gpt_serial::<1>().set(&mut ccm, clock_gate::ON);

    // GPT1 counts with this frequency:
    const GPT1_FREQUENCY_HZ: u32 = PERCLK_CLK_FREQUENCY_HZ / GPT1_DIVIDER;
    const GPT1_DIVIDER: u32 = 100;
    const GPT1_CLOCK_SOURCE: ClockSource = ClockSource::HighFrequencyReferenceClock;

    gpt1.set_divider(GPT1_DIVIDER);
    gpt1.set_clock_source(GPT1_CLOCK_SOURCE);

    let mut count_down = timer::CountDown::<_, GPT1_FREQUENCY>::new(
        timer::RawCountDown::from_gpt(gpt1)
    );

    count_down.start(Megahertz::<u32>::from_raw(3).into_duration());
    // count_down.start(NanosDuration::<u32>::from_raw(33333333));

    let mut pin = gpio2.output(pins.p10);
    let mut ws = Ws2812::new(count_down, pin);

    const NUM_LEDS: usize = 10;
    let mut data = [RGB8::default(); NUM_LEDS];
    for i in 0..NUM_LEDS {
        data[i] = RGB8::new(0, 255, 0);
    }

    loop {
        // led.set_high();
        // delay.delay_ms(DELAY_MS);
        // led.set_low();
        // delay.delay_ms(DELAY_MS);
        // let res = pin.set_high();
        let res = ws.write(data.into_iter());
        match res {
            Ok(value) => {
                led.set_high();
            },
            Err(error) => {
                led.toggle();
                delay.delay_ms(DELAY_MS);
            },
        }

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
