#![no_std]
#![no_main]

mod images;
mod log_setup;
mod mpr121;
mod oled;

use embassy_executor::Spawner;
use embassy_futures::yield_now;
use embassy_time::{Duration, Ticker, Timer};
// use embassy_usb::{
//     UsbDevice,
//     class::cdc_acm::{CdcAcmClass, State},
//     driver::EndpointError,
// };
use log::{error, info};
use panic_halt as _;
use sifli_hal::{
    bind_interrupts,
    i2c::{self, I2c},
    rcc::{Dll, DllStage, Sysclk, Usbsel},
    time::Hertz,
    usart,
    // usb::{Driver, Instance, InterruptHandler},
};
// use static_cell::StaticCell;

use crate::log_setup::init_logger;
use crate::mpr121::{Mpr121, Mpr121Config};
use crate::oled::Oled;

static CHARMBOOT_VERSION: &'static str = env!("CARGO_PKG_VERSION");
static PAGE_SIZE: usize = 4096;

bind_interrupts!(struct Irqs {
    LCDC1 => sifli_hal::lcdc::InterruptHandler<sifli_hal::peripherals::LCDC1>;
    I2C2 => i2c::InterruptHandler<sifli_hal::peripherals::I2C2>;
    I2C3 => i2c::InterruptHandler<sifli_hal::peripherals::I2C3>;
});

// ── Touch bar demo layout ─────────────────────────────────────────────
//
// Each MPR121 drives 8 electrodes (ELE4..ELE11) that we render as vertical
// bars. Device A grows down from the top edge; device B grows up from the
// bottom edge, meeting in the middle.

/// Number of electrodes rendered per device.
const BARS: usize = 8;
/// First electrode used (channels ELE4..ELE11 are the physical bar traces).
const FIRST_ELE: usize = 4;
/// Horizontal pitch of each bar cell (128 / 8 = 16 px).
const BAR_STRIDE: usize = oled::WIDTH / BARS;
/// Filled width of a bar within its cell, leaving a 1 px gutter each side.
const BAR_WIDTH: usize = BAR_STRIDE - 2;
/// Maximum bar height: each device owns half the panel.
const HALF_H: usize = oled::HEIGHT / 2;
/// Fixed full-scale delta (baseline − filtered) that maps to a full-height bar.
/// Tune to taste; smaller = more sensitive.
const MAX_DELTA: i32 = 80;
/// If true, autoscale each frame to the largest delta instead of `MAX_DELTA`.
const AUTO_SCALE: bool = false;

/// Overlay both devices' touch deltas onto `fb` as opposing bar graphs.
///
/// The caller is responsible for the background already in `fb` (e.g. the boot
/// image); this function only turns bar pixels *on*, leaving the rest intact.
///
/// `*_filtered` / `*_baseline` are the 12-channel reads from each MPR121. The
/// per-electrode delta `(baseline << 2) − filtered` sits near zero when
/// untouched and grows with proximity/pressure, independent of trace length.
///
/// Device B is drawn on the top half (bars grow downward from row 0); device A
/// on the bottom half (bars grow upward from the last row). The top device's
/// channel order is reversed so bar position matches the physical touch strip
/// (left touch → left bar) on both boards.
fn draw_bars(
    fb: &mut oled::FrameBuffer,
    a_filtered: &[u16; mpr121::CHANNELS],
    a_baseline: &[u8; mpr121::CHANNELS],
    b_filtered: &[u16; mpr121::CHANNELS],
    b_baseline: &[u8; mpr121::CHANNELS],
) {
    // Delta = (baseline << 2) − filtered for ELE4..11, clamped to >= 0.
    let mut a_delta = [0i32; BARS];
    let mut b_delta = [0i32; BARS];
    let mut max_delta = 1i32;
    for i in 0..BARS {
        let ad =
            (((a_baseline[FIRST_ELE + i] as i32) << 2) - a_filtered[FIRST_ELE + i] as i32).max(0);
        let bd =
            (((b_baseline[FIRST_ELE + i] as i32) << 2) - b_filtered[FIRST_ELE + i] as i32).max(0);
        a_delta[i] = ad;
        b_delta[i] = bd;
        max_delta = max_delta.max(ad).max(bd);
    }

    let scale = if AUTO_SCALE { max_delta } else { MAX_DELTA };

    // Device B: top half, bars grow downward from row 0, reversed.
    for i in 0..BARS {
        let h = (b_delta[BARS - 1 - i].min(scale) as u32 * HALF_H as u32 / scale as u32) as usize;
        let x0 = i * BAR_STRIDE + 1;
        for row in 0..h {
            for col in 0..BAR_WIDTH {
                fb.set_pixel(x0 + col, row, true);
            }
        }
    }

    // Device A: bottom half, bars grow upward from the last row.
    for i in 0..BARS {
        let h = (a_delta[i].min(scale) as u32 * HALF_H as u32 / scale as u32) as usize;
        let x0 = i * BAR_STRIDE + 1;
        for row in 0..h {
            let y = oled::HEIGHT - 1 - row;
            for col in 0..BAR_WIDTH {
                fb.set_pixel(x0 + col, y, true);
            }
        }
    }
}

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

    // ── MPR121 capacitive touch, two devices ────────────────────────────
    //
    // Both MPR121s use the default address 0x5A on separate buses:
    //   Device A = I2C2 (SCL=PA32, SDA=PA33)  — top bars
    //   Device B = I2C3 (SCL=PA30, SDA=PA31)  — bottom bars
    //
    // Both are powered from VDD33_LDO2; enable it and let the rail settle
    // before configuring the chips.
    sifli_hal::pac::PMUC.peri_ldo().modify(|w| {
        w.set_en_vdd33_ldo2(true);
        w.set_vdd33_ldo2_pd(false);
    });
    Timer::after(Duration::from_millis(10)).await;

    let mut i2c_config = i2c::Config::default();
    i2c_config.frequency = Hertz(400_000);

    info!("Initializing MPR121 A (I2C2) and B (I2C3)...");
    let i2c_a = I2c::new(p.I2C2, p.PA32, p.PA33, Irqs, i2c_config);
    let i2c_b = I2c::new(p.I2C3, p.PA30, p.PA31, Irqs, i2c_config);

    let mut dev_a = match Mpr121::new(i2c_a, Mpr121Config::default_12ch()).await {
        Ok(m) => m,
        Err(e) => {
            error!("MPR121 A init failed: {:?}", e);
            loop {
                yield_now().await;
            }
        }
    };
    let mut dev_b = match Mpr121::new(i2c_b, Mpr121Config::default_12ch()).await {
        Ok(m) => m,
        Err(e) => {
            error!("MPR121 B init failed: {:?}", e);
            loop {
                yield_now().await;
            }
        }
    };
    info!("MPR121 A + B configured, rendering touch bars at 60fps");

    let mut fb = oled::FrameBuffer::new();
    let mut ticker = Ticker::every(Duration::from_hz(60));
    loop {
        // Read filtered + baseline from both devices. On a transient bus error,
        // hold the previous frame rather than tearing down the demo.
        let frame = async {
            let a_f = dev_a.read_filtered().await?;
            let a_b = dev_a.read_baseline().await?;
            let b_f = dev_b.read_filtered().await?;
            let b_b = dev_b.read_baseline().await?;
            Ok::<_, mpr121::Mpr121Error>((a_f, a_b, b_f, b_b))
        }
        .await;

        match frame {
            Ok((a_f, a_b, b_f, b_b)) => {
                // Start from the boot image, then overlay the touch bars.
                fb.0.copy_from_slice(&images::BOOT.0);
                draw_bars(&mut fb, &a_f, &a_b, &b_f, &b_b);
                let _ = oled.show(&fb);
            }
            Err(e) => error!("MPR121 read failed: {:?}", e),
        }

        ticker.next().await;
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
