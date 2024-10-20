use crate::{Message, G_KEY, MSG_QUEUE};
use stm32f4xx_hal::gpio::ExtiPin;
use stm32f4xx_hal::pac::interrupt;

// EXTI0 interrupt handles button press
#[interrupt]
fn EXTI0() {
    cortex_m::interrupt::free(|_cs| {
        if let Some(b) = G_KEY.borrow(_cs).borrow_mut().as_mut() {
            b.clear_interrupt_pending_bit();
            MSG_QUEUE.enqueue(Message::ButtonPress('A', 0)).ok();
        }
    });
}
