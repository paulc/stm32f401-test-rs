use crate::{Message, MSG_QUEUE};
use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use heapless;
use stm32f4xx_hal::gpio::alt::otg_fs::{Dm, Dp};
use stm32f4xx_hal::otg_fs::{UsbBus, USB};
use stm32f4xx_hal::pac::interrupt;
use stm32f4xx_hal::pac::{OTG_FS_DEVICE, OTG_FS_GLOBAL, OTG_FS_PWRCLK};
use stm32f4xx_hal::rcc::Clocks;
use usb_device::class_prelude::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

// USB Devices
#[allow(dead_code)]
static mut USB_BUF: [u32; 1024] = [0; 1024];
#[allow(dead_code)]
static mut USB_BUS: Option<UsbBusAllocator<stm32f4xx_hal::otg_fs::UsbBusType>> = None;

type UsbDeviceType = UsbDevice<'static, UsbBus<USB>>;
type UsbSerialType = SerialPort<'static, UsbBus<USB>>;

static G_USB_DEVICE: Mutex<RefCell<Option<UsbDeviceType>>> = Mutex::new(RefCell::new(None));
static G_USB_SERIAL: Mutex<RefCell<Option<UsbSerialType>>> = Mutex::new(RefCell::new(None));

pub fn usb_init(
    periphs: (OTG_FS_GLOBAL, OTG_FS_DEVICE, OTG_FS_PWRCLK),
    pins: (impl Into<Dm>, impl Into<Dp>),
    clocks: &Clocks,
) -> Result<(), &'static str> {
    // Need usb_bus to have static lifetime
    let usb = USB::new(periphs, pins, &clocks);
    unsafe {
        let usb_bus = UsbBus::new(usb, &mut USB_BUF[..]);
        USB_BUS.replace(usb_bus);
    }

    cortex_m::interrupt::free(|cs| {
        if let Some(usb_bus) = unsafe { USB_BUS.as_ref() } {
            let serial = SerialPort::new(&usb_bus);
            let usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
                .device_class(USB_CLASS_CDC)
                .strings(&[StringDescriptors::default()
                    .manufacturer("NA")
                    .product("STM32F401 BlackPill")
                    .serial_number("-blackpill")])
                .unwrap()
                .build();
            // Ensure devices are available in IRQ handler
            G_USB_SERIAL.borrow(cs).replace(Some(serial));
            G_USB_DEVICE.borrow(cs).replace(Some(usb_dev));
        }
    });

    Ok(())
}

#[interrupt]
fn OTG_FS() {
    static mut BUF: heapless::Vec<u8, 64> = heapless::Vec::new();
    cortex_m::interrupt::free(|cs| {
        let mut usb_dev = G_USB_DEVICE.borrow(cs).borrow_mut();
        let mut serial = G_USB_SERIAL.borrow(cs).borrow_mut();
        match (usb_dev.as_mut(), serial.as_mut()) {
            (Some(usb_dev), Some(serial)) => {
                if usb_dev.poll(&mut [serial]) {
                    let mut buf = [0u8; 64];
                    match serial.read(&mut buf) {
                        // Handle USB input - assemble into line and send to MSG_QUEUE
                        Ok(n) => {
                            for i in 0..n {
                                match &buf[i] {
                                    // New Line
                                    b'\n' | b'\r' => {
                                        // CR/NL - send buf to msgbus
                                        serial.write(&[b'\r', b'\n']).ok(); // CR, LF
                                        send_buf_msg(&BUF);
                                        BUF.clear();
                                    }
                                    // DEL, BS
                                    0x7f | 0x08 => {
                                        // Delete char
                                        // NOTE: This doesnt work correctly with unicode chars > 1 byte
                                        //       (can end up with invalid unicode str)
                                        if BUF.pop() != None {
                                            serial.write(&[0x08]).ok(); // BS
                                        }
                                    }
                                    // Any other char
                                    &c => {
                                        serial.write(&[c]).ok();
                                        match BUF.push(buf[i]) {
                                            Ok(_) => {}
                                            Err(_) => {
                                                // Buffer full - send
                                                send_buf_msg(&BUF);
                                                BUF.clear();
                                            }
                                        }
                                    }
                                }
                            }
                            // log::info!("USB Read :: {:?}", &buf[0..n]);
                        }
                        Err(e) => {
                            log::info!("USB Read :: ERR {:?}", e);
                        }
                    }
                }
            }
            _ => log::error!("OTG_FS :: error borrowing (usb_dev,serial)"),
        }
    });
}

fn send_buf_msg(buf: &heapless::Vec<u8, 64>) {
    if let Ok(s) = heapless::String::from_utf8(buf.clone()) {
        if s.len() > 0 {
            MSG_QUEUE.enqueue(Message::SerialInput(s)).ok();
        }
    }
}
