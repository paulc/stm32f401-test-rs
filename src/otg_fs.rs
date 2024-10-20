use crate::{Message, G_USB_DEVICE, G_USB_SERIAL, MSG_QUEUE};
use heapless;
use stm32f4xx_hal::pac::interrupt;

// OTG_FS interrupt handles USB events
fn send_buf_msg(buf: &heapless::Vec<u8, 64>) {
    if let Ok(s) = heapless::String::from_utf8(buf.clone()) {
        if s.len() > 0 {
            MSG_QUEUE.enqueue(Message::SerialInput(s)).ok();
        }
    }
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
