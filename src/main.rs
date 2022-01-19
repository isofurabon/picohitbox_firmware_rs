//
// MIT License

// Copyright (c) 2021 rp-rs organization

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

//! # Pico USB 'Twitchy' Mouse Example
//!
//! Creates a USB HID Class Poiting device (i.e. a virtual mouse) on a Pico
//! board, with the USB driver running in the main thread.
//!
//! It generates movement reports which will twitch the cursor up and down by a
//! few pixels, several times a second.
//!
//! This is a port of
//! https://github.com/atsamd-rs/atsamd/blob/master/boards/itsybitsy_m0/examples/twitching_usb_mouse.rs

#![no_std]
#![no_main]

// The macro for our start-up function
use cortex_m_rt::entry;

// The macro for marking our interrupt functions
use pico::hal::pac::interrupt;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
// use embedded_time::fixed_point::FixedPoint;
// use pico::hal::prelude::*;

use embedded_hal::digital::v2::InputPin;
use rp2040_hal::gpio::{Pin, bank0::*, PullUpInput};


// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use pico::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use pico::hal;

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// USB Human Interface Device (HID) Class support
use usbd_hid::descriptor::generator_prelude::*;
// use usbd_hid::descriptor::MouseReport;
use usbd_hid::hid_class::HIDClass;

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

/// The USB Human Interface Device Driver (shared with the interrupt).
static mut USB_HID: Option<HIDClass<hal::usb::UsbBus>> = None;


// HID descriptor for Gamepad
#[gen_hid_descriptor(
    (collection = APPLICATION, usage_page = GENERIC_DESKTOP, usage = GAMEPAD, ) = {
        (usage_page = BUTTON, usage_min = 1, usage_max = 16) = {
            #[packed_bits 16] #[item_settings data,variable,absolute,not_null,no_wrap,linear] buttons = input;
        };
    }
)]
struct GamepadReport {
    buttons: [u8; 2],
}

// HID descriptor for GamePad
struct GamePad {
    // left, up, right, down
    btnl: Pin<Gpio11, PullUpInput>,
    btnd: Pin<Gpio10, PullUpInput>,
    btnr: Pin<Gpio9, PullUpInput>,
    btnu: Pin<Gpio8, PullUpInput>,

    // button 1 ~ 8
    btn1: Pin<Gpio3, PullUpInput>,
    btn2: Pin<Gpio2, PullUpInput>,
    btn3: Pin<Gpio1, PullUpInput>,
    btn4: Pin<Gpio0, PullUpInput>,
    btn5: Pin<Gpio7, PullUpInput>,
    btn6: Pin<Gpio6, PullUpInput>,
    btn7: Pin<Gpio5, PullUpInput>,
    btn8: Pin<Gpio4, PullUpInput>,

    // option button 1 ~ 4
    opt1: Pin<Gpio15, PullUpInput>,
    opt2: Pin<Gpio14, PullUpInput>,
    opt3: Pin<Gpio13, PullUpInput>,
    opt4: Pin<Gpio12, PullUpInput>,
}

// The USB HID Class driver
impl GamePad {

    fn get_input(&self) -> [u8; 2] {
        let mut state: u16 = 0;

        state |= self.get_hat_input();
        state |= self.get_btn_input();
        state |= self.get_opt_input();

        // convert u16 -> u8;2
        state.to_le_bytes()
    } 

    // get hat input
    fn get_hat_input(&self) -> u16 {
        let mut state: u16 = 0;

        if self.btnl.is_low().unwrap() {
            state |= 1_u16;
        }

        if self.btnd.is_low().unwrap() {
            state |= 1_u16 << 1;
        }

        if self.btnr.is_low().unwrap() {
            state |= 1_u16 << 2;
        }

        if self.btnu.is_low().unwrap() {
            state |= 1_u16 << 3;
        }

        self.soc_cleaner(state)
    }

    // get button input
    fn soc_cleaner(&self, hat_state: u16) -> u16 {
        let mut state = hat_state;

        // left and right, up and down
        let lr = 1_u16 | (1_u16 << 2);
        let ud = (1_u16 << 1) | (1_u16 << 3);

        // if left and right are pressed, ignore both of them.
        if (state & lr) == lr {
            state &= !lr;
        }

        // if up and down are pressed, input up.
        if (state & ud) == ud {
            state &= !(1_u16 << 1);
        }

        state
    }

    // get button input
    fn get_btn_input(&self) -> u16 {
        let mut state: u16 = 0;

        if self.btn1.is_low().unwrap() {
            state |= 1_u16 << 4;
        }

        if self.btn2.is_low().unwrap() {
            state |= 1_u16 << 5;
        }

        if self.btn3.is_low().unwrap() {
            state |= 1_u16 << 6;
        }

        if self.btn4.is_low().unwrap() {
            state |= 1_u16 << 7;
        }

        if self.btn5.is_low().unwrap() {
            state |= 1_u16 << 8;
        }

        if self.btn6.is_low().unwrap() {
            state |= 1_u16 << 9;
        }

        if self.btn7.is_low().unwrap() {
            state |= 1_u16 << 10;
        }

        if self.btn8.is_low().unwrap() {
            state |= 1_u16 << 11;
        }

        state
    }

    // get option button input
    fn get_opt_input(&self) -> u16 {
        let mut state: u16 = 0;

        if self.opt1.is_low().unwrap() {
            state |= 1_u16 << 12;
        }

        if self.opt2.is_low().unwrap() {
            state |= 1_u16 << 13;
        }

        if self.opt3.is_low().unwrap() {
            state |= 1_u16 << 14;
        }

        if self.opt4.is_low().unwrap() {
            state |= 1_u16 << 15;
        }

        state
    }
}

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then submits cursor movement
/// updates periodically.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_BUS = Some(usb_bus);
    }

    // Grab a reference to the USB Bus allocator. We are promising to the
    // compiler not to take mutable access to this global variable whilst this
    // reference exists!
    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };

    // Set up the USB HID Class Device driver, providing Mouse Reports
    let usb_hid = HIDClass::new(bus_ref, GamepadReport::desc(), 1);
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet.
        USB_HID = Some(usb_hid);
    }

    // Create a USB device with a fake VID and PID
    let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x0F0D, 0x0011 /* HORI RAP3 */))
        .manufacturer("isofurabonjour")
        .product("pico hitbox")
        .device_class(3) // HID
        .build();
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_DEVICE = Some(usb_dev);
    }

    unsafe {
        // Enable the USB interrupt
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    };

    // Init pins
    let sio = hal::Sio::new(pac.SIO);
    let pins = pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS
    );

    let gamepad = GamePad {
        btnl: pins.gpio11.into_pull_up_input(),
        btnd: pins.gpio10.into_pull_up_input(),
        btnr: pins.gpio9.into_pull_up_input(),
        btnu: pins.gpio8.into_pull_up_input(),

        btn1: pins.gpio3.into_pull_up_input(),
        btn2: pins.gpio2.into_pull_up_input(),
        btn3: pins.gpio1.into_pull_up_input(),
        btn4: pins.gpio0.into_pull_up_input(),
        btn5: pins.gpio7.into_pull_up_input(),
        btn6: pins.gpio6.into_pull_up_input(),
        btn7: pins.gpio5.into_pull_up_input(),
        btn8: pins.gpio4.into_pull_up_input(),

        opt1: pins.gpio15.into_pull_up_input(),
        opt2: pins.gpio14.into_pull_up_input(),
        opt3: pins.gpio13.into_pull_up_input(),
        opt4: pins.gpio12.into_pull_up_input(),
    };

    loop {
        let report = GamepadReport {
            buttons: gamepad.get_input(),
        };
        push_gamepad_movement(report).ok().unwrap_or(0);
    }
}

/// Submit a new mouse movement report to the USB stack.
///
/// We do this with interrupts disabled, to avoid a race hazard with the USB IRQ.
fn push_gamepad_movement(report: GamepadReport) -> Result<usize, usb_device::UsbError> {
    cortex_m::interrupt::free(|_| unsafe {
        // Now interrupts are disabled, grab the global variable and, if
        // available, send it a HID report
        USB_HID.as_mut().map(|hid| hid.push_input(&report))
    })
    .unwrap()
}

/// This function is called whenever the USB Hardware generates an Interrupt
/// Request.
#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    // Handle USB request
    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let usb_hid = USB_HID.as_mut().unwrap();
    usb_dev.poll(&mut [usb_hid]);
}

// End of file