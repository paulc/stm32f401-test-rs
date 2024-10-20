use crate::{LedState, G_LED, G_LED_TIMER, LED_QUEUE};
use stm32f4xx_hal::pac::interrupt;

// TIM3 interrupt handles LED
#[interrupt]
fn TIM3() {
    static mut LED_STATE: LedState = LedState::Off;;
    cortex_m::interrupt::free(|cs| {
        if let Some(t) = G_LED_TIMER.borrow(cs).borrow_mut().as_mut() {
            let _ = t.wait();
        }
        // Handle messages
        if let Some(m) = LED_QUEUE.dequeue() {
            *LED_STATE = m;
        }
        if let Some(led) = G_LED.borrow(cs).borrow_mut().as_mut() {
            match LED_STATE {
                LedState::On => led.set_low(),
                LedState::Off => led.set_high(),
                LedState::Flash => led.toggle(),
            }
        }
    });
}
