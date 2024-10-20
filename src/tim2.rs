use crate::{LedState, Message, G_QUEUE_TIMER, LED_QUEUE, MSG_QUEUE, TICK_HZ};
use stm32f4xx_hal::pac::interrupt;

// TIM2 interrupt handles Message Queue
#[interrupt]
fn TIM2() {
    static mut COUNTER: u32 = 0;
    cortex_m::interrupt::free(|cs| {
        if let Some(t) = G_QUEUE_TIMER.borrow(cs).borrow_mut().as_mut() {
            let _ = t.wait();
        }
        // Handle messages
        if let Some(m) = MSG_QUEUE.dequeue() {
            log::info!("MESSAGE :: {:?}", m);
            match m {
                Message::ButtonPress(p, n) => {
                    log::info!(">> BUTTON_PRESS: {}{}", p, n);
                    LED_QUEUE.enqueue(LedState::Off).ok();
                }
                Message::SerialInput(s) => match s.as_str() {
                    "LED ON" => {
                        LED_QUEUE.enqueue(LedState::On).ok();
                    }
                    "LED OFF" => {
                        LED_QUEUE.enqueue(LedState::Off).ok();
                    }
                    "LED FLASH" => {
                        LED_QUEUE.enqueue(LedState::Flash).ok();
                    }
                    _ => {}
                },
            }
        }
        *COUNTER += 1;
        if *COUNTER % TICK_HZ == 0 {
            log::info!("[TIMER]");
        }
    });
}
