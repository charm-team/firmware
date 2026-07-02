//! Minimal SSD1312 OLED driver (128x64, 1bpp) over the LCDC peripheral.
//!
//! The SSD1312 is driven in **4-wire SPI mode** (`CS`, `D/C`, `CLK`, `DAT` + a
//! dedicated `RES` line). The SF32LB52x has no general-purpose SPI master, so we
//! use the on-chip LCD controller (`LCDC1`), which speaks 4-wire SPI in
//! hardware.
//!
//! The init sequence and data path are copied byte-for-byte from the known-good
//! reference implementation (rcard-legacy `firmware/sysmodule/display`), assuming
//! a 128x64 panel and otherwise the default `DisplayConfiguration`.
//!
//! **D/C handling (important):** in SSD1312 4-wire SPI, *only* GDDRAM pixel data
//! is sent with `D/C` high. Command opcodes **and their argument bytes** are all
//! sent with `D/C` low. So every byte of the init stream goes out as a command
//! (`send_cmd`); only the per-page pixel bytes go out as data (`send_cmd_data`).
//!
//! The LCDC SPI signals are hard-wired to a fixed pin group on this chip:
//!
//! | Signal      | Pin  |
//! |-------------|------|
//! | `RES` (RSTB)| PA0  |
//! | `CS`        | PA3  |
//! | `CLK`       | PA4  |
//! | `DAT` (DIO0)| PA5  |
//! | `D/C` (DIO1)| PA6  |

use embassy_time::{Duration, Timer};
use sifli_hal::Peripheral;
use sifli_hal::interrupt::typelevel::Binding;
use sifli_hal::lcdc::{
    self, Config, FrequencyConfig, InputColorFormat, InterruptHandler, Lcdc, OutputColorFormat,
    Spi, SpiClkPin, SpiConfig, SpiCsPin, SpiDio0Pin, SpiDio1Pin, SpiLineMode, SpiRstbPin,
};
use sifli_hal::peripherals::LCDC1;

/// Display width in pixels.
pub const WIDTH: usize = 128;
/// Display height in pixels.
pub const HEIGHT: usize = 64;
/// Number of 8-pixel-tall pages (the SSD1312 GDDRAM is page-addressed).
const PAGES: usize = HEIGHT / 8;
/// Size of a full packed framebuffer in bytes.
pub const FRAME_SIZE: usize = WIDTH * PAGES;

/// SPI clock divider. Matches the reference (`CLK_DIV = 4`): SPI clock = LCDC
/// source clock / 4.
const SPI_CLK_DIV: u8 = 4;

/// A packed 1bpp framebuffer for the SSD1312.
///
/// Byte layout matches the panel's page-addressed GDDRAM: byte index
/// `page * WIDTH + x` holds an 8-pixel-tall column slice, where bit `n`
/// (LSB = top) is the pixel at row `page * 8 + n`. This is the same layout used
/// by the reference driver and the common SSD1306/SSD1309 family.
#[repr(C, align(4))]
pub struct FrameBuffer(pub [u8; FRAME_SIZE]);

impl FrameBuffer {
    /// Create a new, all-pixels-off framebuffer.
    pub const fn new() -> Self {
        Self([0; FRAME_SIZE])
    }

    /// Turn every pixel off (`false`) or on (`true`).
    pub fn clear(&mut self, on: bool) {
        self.0.fill(if on { 0xFF } else { 0x00 });
    }

    /// Set a single pixel. Coordinates outside the display are ignored.
    pub fn set_pixel(&mut self, x: usize, y: usize, on: bool) {
        if x >= WIDTH || y >= HEIGHT {
            return;
        }
        let idx = (y / 8) * WIDTH + x;
        let bit = 1u8 << (y % 8);
        if on {
            self.0[idx] |= bit;
        } else {
            self.0[idx] &= !bit;
        }
    }
}

impl Default for FrameBuffer {
    fn default() -> Self {
        Self::new()
    }
}

/// SSD1312 panel initialisation sequence, copied verbatim from the reference for
/// a 128x64 panel with default `DisplayConfiguration`:
///
/// - contrast `0x7F`
/// - no segment remap (`0xA0`), no COM reversal (`0xC0`)
/// - COM pin config `0x12`
/// - charge pump enabled (`0x8D 0x12`)
/// - normal (non-inverted) display (`0xA6`)
///
/// Every byte here is a command-stream byte (sent with `D/C` low), including the
/// argument bytes — see the module docs.
const INIT: &[u8] = &[
    0xAE, // display off
    0xD5, 0x80, // display clock divide ratio / oscillator frequency
    0xA8, 0x3F, // multiplex ratio = height - 1 = 63
    0xAD, 0x40, // IREF selection: external
    0xD3, 0x00, // display offset = 0
    0x40, // display start line = 0
    0x8D, 0x12, // charge pump: enabled (default)
    0x20, 0x02, // memory addressing mode = page
    0xA1, // segment remap: on (compensates for 180° physical mounting)
    0xC0, // COM output scan direction: normal (default)
    0xDA, 0x12, // COM pins hardware configuration (default)
    0x81, 0x7F, // contrast (default)
    0xD9, 0xF1, // pre-charge period
    0xDB, 0x40, // VCOMH deselect level
    0xA6, // normal (non-inverted) display
    0xA4, // resume display from GDDRAM content
];

/// A driver for an SSD1312 OLED on `LCDC1` in 4-wire SPI mode.
pub struct Oled<'d> {
    lcdc: Lcdc<'d, LCDC1, Spi>,
}

impl<'d> Oled<'d> {
    /// Instantiate the display.
    ///
    /// `rstb`/`cs`/`dc`/`clk`/`dat` are the LCDC SPI pins (see the module-level
    /// table). The controller is reset, configured, and the panel
    /// initialisation sequence is sent, leaving the display on and cleared.
    pub async fn new(
        lcdc: LCDC1,
        irq: impl Binding<<LCDC1 as lcdc::Instance>::Interrupt, InterruptHandler<LCDC1>>,
        rstb: impl Peripheral<P = impl SpiRstbPin<LCDC1>> + 'd,
        cs: impl Peripheral<P = impl SpiCsPin<LCDC1>> + 'd,
        dc: impl Peripheral<P = impl SpiDio1Pin<LCDC1>> + 'd,
        clk: impl Peripheral<P = impl SpiClkPin<LCDC1>> + 'd,
        dat: impl Peripheral<P = impl SpiDio0Pin<LCDC1>> + 'd,
    ) -> Self {
        let config = Config {
            width: WIDTH as u16,
            height: HEIGHT as u16,
            // Pixel data is streamed byte-by-byte via send_cmd_data, so the
            // colour-format fields are unused; they just satisfy Config.
            out_color_format: OutputColorFormat::Rgb332,
            in_color_format: InputColorFormat::Rgb332,
            // Hold RES low for 10ms during the hardware reset pulse.
            reset_lcd_interval_us: 10_000,
            dcache_clean: true,
            interface_config: SpiConfig {
                line_mode: SpiLineMode::FourLine,
                write_frequency: FrequencyConfig::Div(SPI_CLK_DIV),
                ..Default::default()
            },
        };

        let lcdc = Lcdc::new_4line_with_rstb(lcdc, irq, rstb, cs, dc, clk, dat, config);

        let mut oled = Self { lcdc };
        oled.init().await;
        oled
    }

    /// Pulse the reset line and run the panel init sequence (mirrors the
    /// reference: 10ms reset low, 10ms after release, then the command stream).
    async fn init(&mut self) {
        self.lcdc.reset_lcd().await;
        Timer::after(Duration::from_millis(10)).await;

        // Every init byte is a command-stream byte (D/C low).
        for &b in INIT {
            let _ = self.lcdc.send_cmd(b as u32, 1, false);
        }

        // Clear GDDRAM (undefined after reset), then turn the display on — same
        // order as the reference.
        let _ = self.show(&FrameBuffer::new());
        let _ = self.lcdc.send_cmd(0xAF, 1, false); // display on
    }

    /// Push a full 128x64 framebuffer to the display, page by page.
    ///
    /// For each page: set the page address and column 0 (command bytes), then
    /// stream the page's WIDTH pixel bytes as data.
    pub fn show(&mut self, fb: &FrameBuffer) -> Result<(), lcdc::Error> {
        for page in 0..PAGES as u8 {
            self.lcdc.send_cmd((0xB0 | page) as u32, 1, false)?; // set page address
            self.lcdc.send_cmd(0x00, 1, false)?; // lower column = 0
            self.lcdc.send_cmd(0x10, 1, false)?; // upper column = 0

            let start = page as usize * WIDTH;
            for &b in &fb.0[start..start + WIDTH] {
                self.lcdc.send_cmd_data(b as u32, 1, false)?;
            }
        }
        Ok(())
    }
}
