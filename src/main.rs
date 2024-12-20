#![no_std]
#![no_main]

use core::cell::RefCell;
use core::fmt::Write;
use cortex_m::interrupt::Mutex;
use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;
use display_interface_spi::SPIInterface;
use embedded_graphics::{
    draw_target::DrawTarget,
    mono_font::{
        //ascii::FONT_9X18_BOLD,
        MonoTextStyle,
    },
    pixelcolor::Rgb565,
    prelude::*,
    // primitives::{PrimitiveStyle, Triangle},
    text::{Alignment, Text},
};
use embedded_hal_bus::spi::ExclusiveDevice;
use heapless;
use ili9341::{DisplaySize240x320, Ili9341, Orientation};
use panic_rtt_target as _;
use profont::PROFONT_18_POINT;
use stm32f4xx_hal::{
    gpio::{Edge, Input, Output, PA0, PC13},
    pac::{self, Interrupt},
    prelude::*,
    spi::{Mode, NoMiso, Phase, Polarity, Spi},
    timer::{CounterHz, Event, Timer},
};

mod exti0;
mod otg_fs;
mod tim2;
mod tim3;

use otg_fs::usb_init;

// MSG_QUEUE
#[derive(Debug)]
enum Message {
    SerialInput(heapless::String<64>),
    ButtonPress(char, u8),
}
static MSG_QUEUE: heapless::mpmc::Q4<Message> = heapless::mpmc::Q4::new();

// LED MSG_QUEUE
enum LedState {
    On,
    Off,
    Flash,
}
static LED_QUEUE: heapless::mpmc::Q2<LedState> = heapless::mpmc::Q2::new();

type Key = PA0<Input>;
static G_KEY: Mutex<RefCell<Option<Key>>> = Mutex::new(RefCell::new(None));

// Timers
type QueueTimer = CounterHz<pac::TIM2>;
static G_QUEUE_TIMER: Mutex<RefCell<Option<QueueTimer>>> = Mutex::new(RefCell::new(None));
type LedTimer = CounterHz<pac::TIM3>;
static G_LED_TIMER: Mutex<RefCell<Option<LedTimer>>> = Mutex::new(RefCell::new(None));

const TICK_HZ: u32 = 50;
const LED_HZ: u32 = 10;

// LED
type Led = PC13<Output>;
static G_LED: Mutex<RefCell<Option<Led>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    // Initialise RTT
    rtt_log::init();
    log::info!("STARTING");

    // Syetem preipherals
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::peripheral::Peripherals::take().unwrap();

    // Need 48MHz PLL for USB
    let rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .use_hse(25.MHz())
        .sysclk(48.MHz())
        .require_pll48clk()
        .freeze();

    // Needed for interrupt handler
    let mut syscfg = dp.SYSCFG.constrain();
    let mut exti = dp.EXTI;

    // delay using system timer
    let mut delay = cp.SYST.delay(&clocks);

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    // ILI9341 LCD
    let lcd_clk = gpiob.pb13.into_alternate();
    let lcd_miso = NoMiso::new();
    let lcd_mosi = gpiob.pb15.into_alternate().internal_pull_up(true);

    let mode = Mode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition,
    };

    let lcd_dc = gpiob.pb0.into_push_pull_output();
    let lcd_cs = gpiob.pb1.into_push_pull_output();
    let lcd_reset = gpiob.pb2.into_push_pull_output();
    let mut lcd_backlight = gpiob.pb12.into_push_pull_output();

    let spi2 = Spi::new(
        dp.SPI2,
        (lcd_clk, lcd_miso, lcd_mosi),
        mode,
        24.MHz(),
        &clocks,
    );
    let spi_delay = dp.TIM11.delay_us(&clocks);
    let spi_device = ExclusiveDevice::new(spi2, lcd_cs, spi_delay).unwrap();
    let display_if = SPIInterface::new(spi_device, lcd_dc);

    log::info!("Configuring Display");

    // Using Ili9341 driver
    let mut display = Ili9341::new(
        display_if,
        lcd_reset,
        &mut delay,
        Orientation::Landscape,
        DisplaySize240x320,
    )
    .unwrap();

    /*
        // Using mipidsi driver

        use mipidsi::{
            models::ILI9341Rgb565,
            options::{ColorOrder, Orientation, Rotation},
            Builder,
        };

        let mut display = Builder::new(ILI9341Rgb565, display_if)
            .reset_pin(lcd_reset)
            .display_size(240, 320)
            .orientation(Orientation::new().rotate(Rotation::Deg90).flip_vertical())
            .color_order(ColorOrder::Bgr)
            .init(&mut delay)
            .unwrap();
    */

    lcd_backlight.set_high();

    test_draw(&mut display);

    // KEY button
    let mut key = gpioa.pa0.into_pull_up_input();

    // Enable interrupt
    key.make_interrupt_source(&mut syscfg);
    key.trigger_on_edge(&mut exti, Edge::Falling);
    key.enable_interrupt(&mut exti);

    // LED
    let mut led = gpioc.pc13.into_push_pull_output();
    led.set_high();

    // TIMERs
    let mut q_timer = Timer::new(dp.TIM2, &clocks).counter_hz();
    q_timer.start(TICK_HZ.Hz()).unwrap();
    q_timer.listen(Event::Update); // Generate an interrupt when the timer expires

    let mut led_timer = Timer::new(dp.TIM3, &clocks).counter_hz();
    led_timer.start(LED_HZ.Hz()).unwrap();
    led_timer.listen(Event::Update); // Generate an interrupt when the timer expires

    // USB
    usb_init(
        (dp.OTG_FS_GLOBAL, dp.OTG_FS_DEVICE, dp.OTG_FS_PWRCLK),
        (gpioa.pa11, gpioa.pa12),
        &clocks,
    )
    .unwrap();

    // Move devices to static globals to allow access from IRQ
    cortex_m::interrupt::free(|cs| {
        // Button
        G_KEY.borrow(cs).replace(Some(key));

        // LED
        G_LED.borrow(cs).replace(Some(led));

        // Timer
        G_QUEUE_TIMER.borrow(cs).replace(Some(q_timer));
        G_LED_TIMER.borrow(cs).replace(Some(led_timer));
    });

    // Unmask interrupts
    unsafe {
        NVIC::unmask(Interrupt::OTG_FS);
        NVIC::unmask(Interrupt::EXTI0);
        NVIC::unmask(Interrupt::TIM2);
        NVIC::unmask(Interrupt::TIM3);
    }

    loop {
        // For debugging use delay rather than WFI (supresses RTT)
        // cortex_m::asm::wfi();
        delay.delay_ms(1u32);
    }
}

fn test_draw<D>(display: &mut D)
where
    D: DrawTarget<Color = Rgb565>,
{
    display.clear(Rgb565::CYAN).ok();

    let mut msg = heapless::String::<30>::new();
    let mut count = 0_u32;

    for _ in 0..5 {
        for i in 0..10 {
            msg.clear();
            write!(msg, "{:4} :: {}", count, "ABCDEFGHIJKLMNOP").ok();

            Text::with_alignment(
                msg.as_str(),
                Point::new(10, i * 24),
                MonoTextStyle::new(&PROFONT_18_POINT, Rgb565::RED),
                Alignment::Left,
            )
            .draw(display)
            .ok();
            count += 1;
        }
        display.clear(Rgb565::CYAN).ok();
    }

    display.clear(Rgb565::BLUE).ok();

    /*
    Text::with_alignment(
        "Hello!",
        Point::new(10, 10),
        MonoTextStyle::new(&FONT_9X18_BOLD, Rgb565::RED),
        Alignment::Left,
    )
    .draw(display)
    .ok();

    Text::with_alignment(
        "There!",
        Point::new(10, 30),
        MonoTextStyle::new(&FONT_9X18_BOLD, Rgb565::YELLOW),
        Alignment::Left,
    )
    .draw(display)
    .ok();

    Triangle::new(
        Point::new(160, 40),
        Point::new(80, 160),
        Point::new(240, 160),
    )
    .into_styled(PrimitiveStyle::with_stroke(Rgb565::RED, 4))
    .draw(display)
    .ok();
    */

    // display.clear(Rgb565::BLACK).ok();
}
