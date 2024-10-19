#![no_std]
#![no_main]

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;
use display_interface_spi::SPIInterface;
use embedded_graphics::{
    draw_target::DrawTarget,
    mono_font::{
        ascii::{FONT_6X12, FONT_8X13, FONT_9X18_BOLD},
        MonoTextStyle,
    },
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyle, Triangle},
    text::{Alignment, Text},
};
use embedded_hal_bus::spi::ExclusiveDevice;
use heapless;
use mipidsi::{
    models::ILI9341Rgb565,
    options::{ColorOrder, Orientation, Rotation},
    Builder,
};
use panic_rtt_target as _;
use stm32f4xx_hal::{
    gpio::{Edge, Input, Output, PA0, PC13},
    otg_fs::{UsbBus, USB},
    pac::{self, interrupt, Interrupt},
    prelude::*,
    spi::{Mode, NoMiso, Phase, Polarity, Spi},
    timer::{CounterHz, Event, Timer},
};
use usb_device::class_prelude::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

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

// USB Devices
type UsbDeviceType = UsbDevice<'static, UsbBus<USB>>;
type UsbSerialType = SerialPort<'static, UsbBus<USB>>;
static G_USB_DEVICE: Mutex<RefCell<Option<UsbDeviceType>>> = Mutex::new(RefCell::new(None));
static G_USB_SERIAL: Mutex<RefCell<Option<UsbSerialType>>> = Mutex::new(RefCell::new(None));

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
    // Static
    static mut USB_BUF: [u32; 1024] = [0; 1024];
    static mut USB_BUS: Option<UsbBusAllocator<UsbBus<USB>>> = None;

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
    let spi_delay = dp.TIM1.delay_us(&clocks);
    let spi_device = ExclusiveDevice::new(spi2, lcd_cs, spi_delay).unwrap();
    let display_if = SPIInterface::new(spi_device, lcd_dc);

    log::info!("Configuring Display");
    let mut display = Builder::new(ILI9341Rgb565, display_if)
        .reset_pin(lcd_reset)
        .display_size(240, 320)
        .orientation(Orientation::new().rotate(Rotation::Deg90).flip_vertical())
        .color_order(ColorOrder::Bgr)
        .init(&mut delay)
        .unwrap();

    lcd_backlight.set_high();
    display.clear(Rgb565::CYAN).unwrap();

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

    // Setup USB
    let usb = USB::new(
        (dp.OTG_FS_GLOBAL, dp.OTG_FS_DEVICE, dp.OTG_FS_PWRCLK),
        (gpioa.pa11, gpioa.pa12),
        &clocks,
    );

    // Need usb_bus to have static lifetime due to refs in IRQs
    USB_BUS.replace(UsbBus::new(usb, &mut USB_BUF[..]));

    // Move devices to static globals to allow access from IRQ
    cortex_m::interrupt::free(|cs| {
        // Button
        G_KEY.borrow(cs).replace(Some(key));

        // LED
        G_LED.borrow(cs).replace(Some(led));

        // Timer
        G_QUEUE_TIMER.borrow(cs).replace(Some(q_timer));
        G_LED_TIMER.borrow(cs).replace(Some(led_timer));

        // USB
        if let Some(usb_bus) = USB_BUS.as_ref() {
            let serial = SerialPort::new(usb_bus);

            let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
                .device_class(USB_CLASS_CDC)
                .strings(&[StringDescriptors::default()
                    .manufacturer("NA")
                    .product("STM32F401 BlackPill")
                    .serial_number("-blackpill")])
                .unwrap()
                .build();

            G_USB_SERIAL.borrow(cs).replace(Some(serial));
            G_USB_DEVICE.borrow(cs).replace(Some(usb_dev));
        }
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
}

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

fn send_buf_msg(buf: &heapless::Vec<u8, 64>) {
    if let Ok(s) = heapless::String::from_utf8(buf.clone()) {
        if s.len() > 0 {
            MSG_QUEUE.enqueue(Message::SerialInput(s)).ok();
        }
    }
}

// OTG_FS interrupt handles USB events
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
