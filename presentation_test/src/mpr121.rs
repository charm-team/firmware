//! Async MPR121 capacitive touch controller driver.
//!
//! The MPR121 is a 12-channel capacitive touch sensor on I2C. This driver is a
//! port of the legacy `rcard` MPR121 sysmodule (`legacy/mpr121.rs` +
//! `legacy/mpr121_api.rs`), rebuilt around embassy patterns and the sifli-hal
//! I2C implementation instead of the old IPC/userlib framework.
//!
//! The configuration types (`Mpr121Config` and friends) are kept faithful to the
//! reference, minus its IPC/`zerocopy`/`serde` derive machinery — here they are
//! plain `Copy` structs with `const` constructors so a config can be built in a
//! `const` context.
//!
//! All register access happens over the [`sifli_hal::i2c`] async driver:
//! register writes are a two-byte `write` (`[reg, val]`), and reads use
//! `write_read` with the MPR121's register auto-increment to pull a contiguous
//! block in one transaction.
//!
//! # Example
//! ```ignore
//! let i2c = I2c::new(p.I2C2, p.PA32, p.PA33, Irqs, i2c::Config::default());
//! let mut mpr = Mpr121::new(i2c, Mpr121Config::default_12ch()).await?;
//! let touched = mpr.touch_status().await?;
//! ```

#![allow(dead_code)]

use sifli_hal::i2c::{self, I2c};
use sifli_hal::mode::Async;

/// Default 7-bit I2C address (ADDR pin tied to VSS).
pub const DEFAULT_ADDRESS: u8 = 0x5A;

// ── MPR121 registers ──────────────────────────────────────────────────

const REG_TOUCH_STATUS_L: u8 = 0x00;
const REG_TOUCH_STATUS_H: u8 = 0x01;
const REG_FILTERED_DATA_BASE: u8 = 0x04;
const REG_BASELINE_BASE: u8 = 0x1E;

const REG_MHD_RISING: u8 = 0x2B;
const REG_NHD_RISING: u8 = 0x2C;
const REG_NCL_RISING: u8 = 0x2D;
const REG_FDL_RISING: u8 = 0x2E;
const REG_MHD_FALLING: u8 = 0x2F;
const REG_NHD_FALLING: u8 = 0x30;
const REG_NCL_FALLING: u8 = 0x31;
const REG_FDL_FALLING: u8 = 0x32;
const REG_NHD_TOUCHED: u8 = 0x33;
const REG_NCL_TOUCHED: u8 = 0x34;
const REG_FDL_TOUCHED: u8 = 0x35;

const REG_TOUCH_THRESHOLD_BASE: u8 = 0x41;

const REG_DEBOUNCE: u8 = 0x5B;
const REG_AFE_CONFIG1: u8 = 0x5C;
const REG_AFE_CONFIG2: u8 = 0x5D;
const REG_ECR: u8 = 0x5E;
const REG_AUTOCONFIG0: u8 = 0x7B;
const REG_AUTOCONFIG1: u8 = 0x7C;
const REG_AUTOCONFIG_USL: u8 = 0x7D;
const REG_AUTOCONFIG_LSL: u8 = 0x7E;
const REG_AUTOCONFIG_TL: u8 = 0x7F;
const REG_SOFT_RESET: u8 = 0x80;

/// Soft-reset magic value written to [`REG_SOFT_RESET`].
const SOFT_RESET_MAGIC: u8 = 0x63;

/// Number of physical electrode channels.
pub const CHANNELS: usize = 12;

// ── Error ─────────────────────────────────────────────────────────────

/// An error from the MPR121 driver.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum Mpr121Error {
    /// The underlying I2C transfer failed.
    I2c(i2c::Error),
    /// A configured electrode count exceeded [`CHANNELS`].
    InvalidElectrodeCount,
    /// The touch-status register reported an over-current fault (bit 15).
    OverCurrent,
}

impl From<i2c::Error> for Mpr121Error {
    fn from(e: i2c::Error) -> Self {
        Mpr121Error::I2c(e)
    }
}

// ── Baseline filtering (registers 0x2B–0x35) ─────────────────────────

/// Baseline filter parameters for rising, falling, and touched scenarios.
///
/// Controls how the MPR121 tracks slow background capacitance drift.
#[derive(Clone, Copy, Debug)]
pub struct BaselineFilter {
    /// Max half delta, rising (1–63)
    pub mhd_rising: u8,
    /// Noise half delta amount, rising (1–63)
    pub nhd_rising: u8,
    /// Noise count limit, rising (0–255)
    pub ncl_rising: u8,
    /// Filter delay count limit, rising (0–255)
    pub fdl_rising: u8,
    /// Max half delta, falling (1–63)
    pub mhd_falling: u8,
    /// Noise half delta amount, falling (1–63)
    pub nhd_falling: u8,
    /// Noise count limit, falling (0–255)
    pub ncl_falling: u8,
    /// Filter delay count limit, falling (0–255)
    pub fdl_falling: u8,
    /// Noise half delta amount, touched (1–63)
    pub nhd_touched: u8,
    /// Noise count limit, touched (0–255)
    pub ncl_touched: u8,
    /// Filter delay count limit, touched (0–255)
    pub fdl_touched: u8,
}

impl BaselineFilter {
    /// Datasheet-recommended defaults.
    pub const fn default() -> Self {
        Self {
            mhd_rising: 0x01,
            nhd_rising: 0x01,
            ncl_rising: 0x0E,
            fdl_rising: 0x00,
            mhd_falling: 0x01,
            nhd_falling: 0x05,
            ncl_falling: 0x01,
            fdl_falling: 0x00,
            nhd_touched: 0x00,
            ncl_touched: 0x00,
            fdl_touched: 0x00,
        }
    }
}

// ── Thresholds ────────────────────────────────────────────────────────

/// Global touch/release thresholds applied to all enabled electrodes.
///
/// Touch fires when `baseline − filtered > touch`; release fires when
/// `baseline − filtered < release`. Typical range: touch 4–16, release
/// slightly less than touch.
#[derive(Clone, Copy, Debug)]
pub struct ThresholdConfig {
    pub touch: u8,
    pub release: u8,
}

impl ThresholdConfig {
    pub const fn new(touch: u8, release: u8) -> Self {
        Self { touch, release }
    }
}

// ── Debounce (register 0x5B) ──────────────────────────────────────────

/// Consecutive detections required before a touch/release status change takes
/// effect. Higher values reject noise at the cost of latency
/// (`delay = ESI × SFI × debounce`).
#[derive(Clone, Copy, Debug)]
#[repr(u8)]
pub enum Debounce {
    Off = 0,
    Count1 = 1,
    Count2 = 2,
    Count3 = 3,
    Count4 = 4,
    Count5 = 5,
    Count6 = 6,
    Count7 = 7,
}

/// Separate debounce counts for touch and release transitions.
#[derive(Clone, Copy, Debug)]
pub struct DebounceConfig {
    pub touch: Debounce,
    pub release: Debounce,
}

impl DebounceConfig {
    pub const fn off() -> Self {
        Self {
            touch: Debounce::Off,
            release: Debounce::Off,
        }
    }
}

// ── AFE / filter config (registers 0x5C–0x5D) ────────────────────────

/// First filter iterations — number of ADC samples averaged in the first-level
/// filter (register 0x5C bits [7:6]).
#[derive(Clone, Copy, Debug)]
#[repr(u8)]
pub enum FirstFilterIterations {
    Samples6 = 0,
    Samples10 = 1,
    Samples18 = 2,
    Samples34 = 3,
}

/// Global charge/discharge time per measurement (register 0x5D bits [7:5]).
/// Time = `2^(n−2)` µs.
#[derive(Clone, Copy, Debug)]
#[repr(u8)]
pub enum ChargeDischargeTime {
    Disabled = 0,
    Us0_5 = 1,
    Us1 = 2,
    Us2 = 3,
    Us4 = 4,
    Us8 = 5,
    Us16 = 6,
    Us32 = 7,
}

/// Second filter iterations — number of samples for the second-level filter
/// (register 0x5D bits [4:3]).
#[derive(Clone, Copy, Debug)]
#[repr(u8)]
pub enum SecondFilterIterations {
    Samples4 = 0,
    Samples6 = 1,
    Samples10 = 2,
    Samples18 = 3,
}

/// Electrode sample interval — period between second-level filter samples
/// (register 0x5D bits [2:0]). Period = `2^n` ms.
#[derive(Clone, Copy, Debug)]
#[repr(u8)]
pub enum SampleInterval {
    Ms1 = 0,
    Ms2 = 1,
    Ms4 = 2,
    Ms8 = 3,
    Ms16 = 4,
    Ms32 = 5,
    Ms64 = 6,
    Ms128 = 7,
}

/// Analog front-end and digital filter configuration.
#[derive(Clone, Copy, Debug)]
pub struct AfeConfig {
    /// First-level filter sample count
    pub ffi: FirstFilterIterations,
    /// Global charge/discharge current in µA (0 = disabled, 1–63)
    pub cdc: u8,
    /// Global charge/discharge time
    pub cdt: ChargeDischargeTime,
    /// Second-level filter sample count
    pub sfi: SecondFilterIterations,
    /// Electrode sample interval
    pub esi: SampleInterval,
}

impl AfeConfig {
    pub const fn default() -> Self {
        Self {
            ffi: FirstFilterIterations::Samples34,
            cdc: 63,
            cdt: ChargeDischargeTime::Us0_5,
            sfi: SecondFilterIterations::Samples10,
            esi: SampleInterval::Ms8,
        }
    }
}

// ── Electrode configuration (register 0x5E) ──────────────────────────

/// Controls baseline tracking and how the initial baseline value is loaded on
/// entering run mode (ECR bits [7:6]).
#[derive(Clone, Copy, Debug)]
#[repr(u8)]
pub enum CalibrationLock {
    /// Tracking enabled, initial = current baseline register value
    TrackingCurrent = 0,
    /// Baseline tracking disabled
    Disabled = 1,
    /// Tracking enabled, initial = 5 high bits of first electrode read
    TrackingFast = 2,
    /// Tracking enabled, initial = full 10 bits of first electrode read
    TrackingFull = 3,
}

/// Proximity detection mode — which electrodes are combined for the 13th
/// proximity channel (ECR bits [5:4]).
#[derive(Clone, Copy, Debug)]
#[repr(u8)]
pub enum ProximityMode {
    Disabled = 0,
    Ele0To1 = 1,
    Ele0To3 = 2,
    Ele0To11 = 3,
}

/// Electrode configuration register fields.
#[derive(Clone, Copy, Debug)]
pub struct ElectrodeConfig {
    /// Baseline tracking mode
    pub calibration: CalibrationLock,
    /// Proximity detection mode
    pub proximity: ProximityMode,
    /// Number of electrodes to enable (0–12)
    pub electrode_count: u8,
}

impl ElectrodeConfig {
    pub const fn new(count: u8) -> Self {
        Self {
            calibration: CalibrationLock::TrackingFast,
            proximity: ProximityMode::Disabled,
            electrode_count: count,
        }
    }
}

// ── Auto-configuration (registers 0x7B–0x7F) ─────────────────────────

/// Number of retries for auto-config before setting OOR (register 0x7B bits [3:2]).
#[derive(Clone, Copy, Debug)]
#[repr(u8)]
pub enum AutoConfigRetry {
    None = 0,
    Retry2 = 1,
    Retry4 = 2,
    Retry8 = 3,
}

/// Auto-configuration settings.
///
/// When enabled, the MPR121 automatically searches for optimal CDC/CDT values
/// per channel on each stop→run transition.
#[derive(Clone, Copy, Debug)]
pub struct AutoConfig {
    /// Enable auto-configuration on stop→run transition
    pub enabled: u8,
    /// Enable auto-reconfiguration for OOR channels
    pub reconfig_enabled: u8,
    /// Skip charge-time (CDT) search
    pub skip_charge_time: u8,
    /// Retry count on failure
    pub retry: AutoConfigRetry,
    /// Upper side limit (8 MSB of 10-bit target, typ. (VDD−0.7)/VDD × 256)
    pub upper_limit: u8,
    /// Target level (typ. USL × 0.9)
    pub target_level: u8,
    /// Lower side limit (typ. USL × 0.65)
    pub lower_limit: u8,
}

impl AutoConfig {
    pub const fn disabled() -> Self {
        Self {
            enabled: 0,
            reconfig_enabled: 0,
            skip_charge_time: 0,
            retry: AutoConfigRetry::None,
            upper_limit: 0,
            target_level: 0,
            lower_limit: 0,
        }
    }

    /// Standard auto-config for a given supply voltage.
    ///
    /// `USL = (VDD − 0.7) / VDD × 256`, `TL = USL × 0.9`, `LSL = USL × 0.65`.
    /// Auto-reconfig (ARE) is disabled: disconnected/OOR electrodes are left
    /// as-is after the initial search rather than triggering continuous retries.
    pub const fn for_vdd_mv(vdd_mv: u16) -> Self {
        let usl = ((vdd_mv as u32 - 700) * 256 / vdd_mv as u32) as u8;
        let tl = (usl as u16 * 9 / 10) as u8;
        let lsl = (usl as u16 * 65 / 100) as u8;
        Self {
            enabled: 1,
            reconfig_enabled: 0,
            skip_charge_time: 0,
            retry: AutoConfigRetry::Retry2,
            upper_limit: usl,
            target_level: tl,
            lower_limit: lsl,
        }
    }
}

// ── Top-level config ──────────────────────────────────────────────────

/// A complete MPR121 configuration.
#[derive(Clone, Copy, Debug)]
pub struct Mpr121Config {
    pub thresholds: ThresholdConfig,
    pub baseline: BaselineFilter,
    pub debounce: DebounceConfig,
    pub afe: AfeConfig,
    pub electrode: ElectrodeConfig,
    pub auto_config: AutoConfig,
}

impl Mpr121Config {
    /// All 12 electrodes enabled, manual (no auto-config) charge settings.
    pub const fn default_12ch() -> Self {
        Self {
            thresholds: ThresholdConfig::new(2, 1),
            baseline: BaselineFilter::default(),
            debounce: DebounceConfig::off(),
            afe: AfeConfig::default(),
            electrode: ElectrodeConfig::new(12),
            auto_config: AutoConfig::disabled(),
        }
    }

    /// All 12 electrodes enabled with 3.3 V auto-configuration.
    pub const fn auto_12ch_3v3() -> Self {
        Self {
            thresholds: ThresholdConfig::new(2, 1),
            baseline: BaselineFilter::default(),
            debounce: DebounceConfig::off(),
            afe: AfeConfig::default(),
            electrode: ElectrodeConfig::new(12),
            auto_config: AutoConfig::for_vdd_mv(3300),
        }
    }
}

// ── Driver ────────────────────────────────────────────────────────────

/// An async MPR121 driver bound to a single async I2C bus.
pub struct Mpr121<'d, T: i2c::Instance> {
    i2c: I2c<'d, T, Async>,
    address: u8,
    config: Mpr121Config,
}

impl<'d, T: i2c::Instance> Mpr121<'d, T> {
    /// Create and configure an MPR121 at [`DEFAULT_ADDRESS`].
    ///
    /// Runs a soft reset followed by the full configuration sequence, leaving
    /// the device in run mode.
    pub async fn new(i2c: I2c<'d, T, Async>, config: Mpr121Config) -> Result<Self, Mpr121Error> {
        Self::new_with_address(i2c, DEFAULT_ADDRESS, config).await
    }

    /// Create and configure an MPR121 at an explicit I2C address.
    pub async fn new_with_address(
        i2c: I2c<'d, T, Async>,
        address: u8,
        config: Mpr121Config,
    ) -> Result<Self, Mpr121Error> {
        let mut dev = Self {
            i2c,
            address,
            config,
        };
        dev.configure().await?;
        Ok(dev)
    }

    /// Write a single configuration register.
    async fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), Mpr121Error> {
        self.i2c.write(self.address, &[reg, val]).await?;
        Ok(())
    }

    /// Read a contiguous register block starting at `start_reg` using the
    /// MPR121's register auto-increment.
    async fn read_block(&mut self, start_reg: u8, buf: &mut [u8]) -> Result<(), Mpr121Error> {
        if buf.is_empty() {
            return Ok(());
        }
        self.i2c.write_read(self.address, &[start_reg], buf).await?;
        Ok(())
    }

    /// Soft-reset the device (returns it to stop mode with default registers).
    pub async fn soft_reset(&mut self) -> Result<(), Mpr121Error> {
        self.write_reg(REG_SOFT_RESET, SOFT_RESET_MAGIC).await
    }

    /// Run-mode ECR value derived from the electrode config.
    fn run_ecr(&self) -> u8 {
        let e = &self.config.electrode;
        ((e.calibration as u8) << 6) | ((e.proximity as u8) << 4) | (e.electrode_count & 0x0F)
    }

    /// Apply the full configuration: soft reset, program all registers in stop
    /// mode, then enter run mode via ECR.
    async fn configure(&mut self) -> Result<(), Mpr121Error> {
        if self.config.electrode.electrode_count > CHANNELS as u8 {
            return Err(Mpr121Error::InvalidElectrodeCount);
        }

        self.soft_reset().await?;

        // After reset, ECR defaults to 0x00 (stop mode). All configuration must
        // happen in stop mode.

        let bl = self.config.baseline;
        self.write_reg(REG_MHD_RISING, bl.mhd_rising).await?;
        self.write_reg(REG_NHD_RISING, bl.nhd_rising).await?;
        self.write_reg(REG_NCL_RISING, bl.ncl_rising).await?;
        self.write_reg(REG_FDL_RISING, bl.fdl_rising).await?;
        self.write_reg(REG_MHD_FALLING, bl.mhd_falling).await?;
        self.write_reg(REG_NHD_FALLING, bl.nhd_falling).await?;
        self.write_reg(REG_NCL_FALLING, bl.ncl_falling).await?;
        self.write_reg(REG_FDL_FALLING, bl.fdl_falling).await?;
        self.write_reg(REG_NHD_TOUCHED, bl.nhd_touched).await?;
        self.write_reg(REG_NCL_TOUCHED, bl.ncl_touched).await?;
        self.write_reg(REG_FDL_TOUCHED, bl.fdl_touched).await?;

        let th = self.config.thresholds;
        for ele in 0..self.config.electrode.electrode_count {
            let base = REG_TOUCH_THRESHOLD_BASE + (ele * 2);
            self.write_reg(base, th.touch).await?;
            self.write_reg(base + 1, th.release).await?;
        }

        let db = self.config.debounce;
        self.write_reg(REG_DEBOUNCE, ((db.release as u8) << 4) | (db.touch as u8))
            .await?;

        let afe = self.config.afe;
        // 0x5C: FFI [7:6] | CDC [5:0]
        self.write_reg(REG_AFE_CONFIG1, ((afe.ffi as u8) << 6) | (afe.cdc & 0x3F))
            .await?;
        // 0x5D: CDT [7:5] | SFI [4:3] | ESI [2:0]
        self.write_reg(
            REG_AFE_CONFIG2,
            ((afe.cdt as u8) << 5) | ((afe.sfi as u8) << 3) | (afe.esi as u8),
        )
        .await?;

        let ac = self.config.auto_config;
        if ac.enabled != 0 {
            // 0x7B: FFI [7:6] | RETRY [5:4] | BVA (=CL) [3:2] | ARE [1] | ACE [0]
            let ac0 = ((afe.ffi as u8) << 6)
                | ((ac.retry as u8) << 4)
                | ((self.config.electrode.calibration as u8) << 2)
                | ((ac.reconfig_enabled & 1) << 1)
                | (ac.enabled & 1);
            self.write_reg(REG_AUTOCONFIG0, ac0).await?;
            // 0x7C: SCTS [7]
            self.write_reg(REG_AUTOCONFIG1, (ac.skip_charge_time & 1) << 7)
                .await?;
            self.write_reg(REG_AUTOCONFIG_USL, ac.upper_limit).await?;
            self.write_reg(REG_AUTOCONFIG_LSL, ac.lower_limit).await?;
            self.write_reg(REG_AUTOCONFIG_TL, ac.target_level).await?;
        }

        // ECR: CL [7:6] | ELEPROX_EN [5:4] | ELE_EN [3:0] — writing enters run mode.
        let ecr = self.run_ecr();
        self.write_reg(REG_ECR, ecr).await?;

        Ok(())
    }

    /// Read the touch-status bitmask (bit `N` = electrode `N` touched).
    ///
    /// Returns [`Mpr121Error::OverCurrent`] if the over-current fault bit is set.
    pub async fn touch_status(&mut self) -> Result<u16, Mpr121Error> {
        let mut raw = [0u8; 2];
        self.read_block(REG_TOUCH_STATUS_L, &mut raw).await?;
        let status = ((raw[1] as u16) << 8) | (raw[0] as u16);
        if status & (1 << 15) != 0 {
            return Err(Mpr121Error::OverCurrent);
        }
        Ok(status & 0x0FFF)
    }

    /// Convenience wrapper: is a specific electrode currently touched?
    pub async fn is_touched(&mut self, electrode: u8) -> Result<bool, Mpr121Error> {
        Ok(self.touch_status().await? & (1 << electrode) != 0)
    }

    /// Read the 10-bit filtered electrode data for all [`CHANNELS`] channels.
    ///
    /// Only the enabled electrodes are read from the device; unread channels
    /// are left as `0`.
    pub async fn read_filtered(&mut self) -> Result<[u16; CHANNELS], Mpr121Error> {
        let count = self.config.electrode.electrode_count as usize;
        let mut raw = [0u8; CHANNELS * 2];
        self.read_block(REG_FILTERED_DATA_BASE, &mut raw[..count * 2])
            .await?;

        let mut out = [0u16; CHANNELS];
        for (i, slot) in out.iter_mut().enumerate().take(count) {
            let lo = raw[i * 2] as u16;
            let hi = raw[i * 2 + 1] as u16;
            *slot = (hi << 8) | lo;
        }
        Ok(out)
    }

    /// Read the 8-bit baseline values for all [`CHANNELS`] channels.
    ///
    /// Left-shift a baseline value by 2 to compare it against 10-bit filtered
    /// data. Only enabled electrodes are read; the rest stay `0`.
    pub async fn read_baseline(&mut self) -> Result<[u8; CHANNELS], Mpr121Error> {
        let count = self.config.electrode.electrode_count as usize;
        let mut raw = [0u8; CHANNELS];
        self.read_block(REG_BASELINE_BASE, &mut raw[..count]).await?;
        Ok(raw)
    }

    /// Set per-electrode touch/release thresholds, overriding the global config.
    ///
    /// The device is briefly returned to stop mode (required to write config
    /// registers) and then back to run mode.
    pub async fn set_threshold(
        &mut self,
        electrode: u8,
        touch: u8,
        release: u8,
    ) -> Result<(), Mpr121Error> {
        if electrode >= self.config.electrode.electrode_count {
            return Err(Mpr121Error::InvalidElectrodeCount);
        }
        // Config registers can only be written in stop mode.
        self.write_reg(REG_ECR, 0x00).await?;
        let base = REG_TOUCH_THRESHOLD_BASE + (electrode * 2);
        self.write_reg(base, touch).await?;
        self.write_reg(base + 1, release).await?;
        // Re-enter run mode.
        let ecr = self.run_ecr();
        self.write_reg(REG_ECR, ecr).await
    }

    /// Soft-reset and reapply the original configuration.
    pub async fn reset(&mut self) -> Result<(), Mpr121Error> {
        self.configure().await
    }

    /// The configuration this device was initialised with.
    pub fn config(&self) -> &Mpr121Config {
        &self.config
    }
}
