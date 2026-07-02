#![no_std]
#![no_main]

mod images;
mod log_setup;
mod oled;

use embassy_executor::Spawner;
use embassy_futures::yield_now;
use embassy_time::Timer;
// use embassy_usb::{
//     UsbDevice,
//     class::cdc_acm::{CdcAcmClass, State},
//     driver::EndpointError,
// };
use log::{error, info};
use panic_halt as _;
use sifli_hal::{
    bind_interrupts,
    i2c,
    rcc::{Dll, DllStage, Sysclk, Usbsel},
    usart,
    // usb::{Driver, Instance, InterruptHandler},
};
// use static_cell::StaticCell;

use crate::log_setup::init_logger;
use crate::oled::Oled;

static CHARMBOOT_VERSION: &'static str = env!("CARGO_PKG_VERSION");
static PAGE_SIZE: usize = 4096;

bind_interrupts!(struct Irqs {
    LCDC1 => sifli_hal::lcdc::InterruptHandler<sifli_hal::peripherals::LCDC1>;
    I2C2 => i2c::InterruptHandler<sifli_hal::peripherals::I2C2>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    // Configure 240MHz system clock using DLL1
    // DLL1 Freq = (stg + 1) * 24MHz = (9 + 1) * 24MHz = 240MHz
    // DLL2 for USB at 240MHz, USB = 240MHz / 4 = 60MHz (required by USB PHY)
    let hal_config = sifli_hal::Config::default().with_rcc(
        const {
            sifli_hal::rcc::ConfigBuilder::new()
                .with_sys(Sysclk::Dll1)
                .with_dll1(Dll::new().with_stg(DllStage::Mul10))
                .with_dll2(Dll::new().with_stg(DllStage::Mul10))
                .with_mux(sifli_hal::rcc::ClockMux::new().with_usbsel(Usbsel::Dll2))
                .checked()
        },
    );

    let p = sifli_hal::init(hal_config);

    let mut config = usart::Config::default();
    config.baudrate = 1000000;
    let usart = usart::Uart::new_blocking(p.USART1, p.PA18, p.PA19, config).unwrap();

    init_logger(usart, log::LevelFilter::Info);

    info!("charmboot! version {}", CHARMBOOT_VERSION);
    sifli_hal::rcc::test_print_clocks();

    // Create the driver, from the HAL
    // let driver = Driver::new(p.USBC, Irqs, p.PA35, p.PA36);

    // Create embassy-usb Config
    // let usb_config = {
    //     let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    //     config.manufacturer = Some("SiFli-rs");
    //     config.product = Some("sifli-rs USB-serial example");
    //     config.serial_number = Some("12345678");
    //     config.max_power = 100;
    //     config.max_packet_size_0 = 64;

    //     // Required for windows compatibility.
    //     // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
    //     config.device_class = 0xEF;
    //     config.device_sub_class = 0x02;
    //     config.device_protocol = 0x01;
    //     config.composite_with_iads = true;
    //     config
    // };

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    // let mut builder = {
    //     static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    //     static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    //     static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

    //     embassy_usb::Builder::new(
    //         driver,
    //         usb_config,
    //         CONFIG_DESCRIPTOR.init([0; 256]),
    //         BOS_DESCRIPTOR.init([0; 256]),
    //         &mut [], // no msos descriptors
    //         CONTROL_BUF.init([0; 64]),
    //     )
    // };

    // Create classes on the builder.
    // let mut class = {
    //     static STATE: StaticCell<State> = StaticCell::new();
    //     let state = STATE.init(State::new());
    //     CdcAcmClass::new(&mut builder, state, 64)
    // };

    // Build the builder.
    // let usb = builder.build();

    // Run the USB device.
    // spawner.spawn(usb_task(usb)).unwrap();

    // Do stuff with the class!
    // loop {
    //     class.wait_connection().await;
    //     info!("Connected");
    //     let _ = echo(&mut class).await;
    //     info!("Disconnected");
    // }

    sifli_hal::pac::PMUC
        .peri_ldo()
        .modify(|r| r.set_vdd33_ldo3_pd(false));
    sifli_hal::pac::PMUC
        .peri_ldo()
        .modify(|r| r.set_en_vdd33_ldo3(true));

    // RST=PA0, CS=PA3, CLK=PA4, DAT=PA5, D/C=PA6
    info!("Initializing OLED...");
    let mut oled = Oled::new(p.LCDC1, Irqs, p.PA0, p.PA3, p.PA6, p.PA4, p.PA5).await;

    oled.show(&images::BOOT).unwrap();
    info!("boot.png displayed");

    loop {
        yield_now().await;
    }
}

// type MyUsbDriver = Driver<'static, sifli_hal::peripherals::USBC>;
// type MyUsbDevice = UsbDevice<'static, MyUsbDriver>;

// #[embassy_executor::task]
// async fn usb_task(mut usb: MyUsbDevice) -> ! {
//     usb.run().await
// }

// struct Disconnected {}

// impl From<EndpointError> for Disconnected {
//     fn from(val: EndpointError) -> Self {
//         match val {
//             EndpointError::BufferOverflow => panic!("Buffer overflow"),
//             EndpointError::Disabled => Disconnected {},
//         }
//     }
// }

// async fn echo<'d, T: Instance + 'd>(
//     class: &mut CdcAcmClass<'d, Driver<'d, T>>,
// ) -> Result<(), Disconnected> {
//     let mut buf = [0; 64];
//     loop {
//         let n = class.read_packet(&mut buf).await?;
//         let data = &buf[..n];
//         info!("data!! yay");
//         class.write_packet(data).await?;
//     }
// }
