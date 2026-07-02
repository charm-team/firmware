#![no_std]

use rcard_log::Format;

macro_rules! mpr_enum {
    ($(#[$meta:meta])* $vis:vis enum $name:ident { $($(#[$vmeta:meta])* $variant:ident = $val:expr),* $(,)? }) => {
        #[derive(
            Clone, Copy, Debug,
            zerocopy::TryFromBytes, zerocopy::IntoBytes,
            zerocopy::KnownLayout, zerocopy::Immutable,
            Format,
            serde::Serialize, serde::Deserialize,
            postcard_schema::Schema,
        )]
        #[repr(u8)]
        $(#[$meta])*
        $vis enum $name { $($(#[$vmeta])* $variant = $val),* }
    };
}

macro_rules! mpr_struct {
    ($(#[$meta:meta])* $vis:vis struct $name:ident { $($(#[$fmeta:meta])* $fvis:vis $field:ident : $ty:ty),* $(,)? }) => {
        #[derive(
            Clone, Copy, Debug,
            zerocopy::TryFromBytes, zerocopy::IntoBytes,
            zerocopy::KnownLayout, zerocopy::Immutable,
            Format,
            serde::Serialize, serde::Deserialize,
            postcard_schema::Schema,
        )]
        #[repr(C, packed)]
        $(#[$meta])*
        $vis struct $name { $($(#[$fmeta])* $fvis $field : $ty),* }
    };
}

// ── Errors ────────────────────────────────────────────────────────────

mpr_enum! {
    pub enum Mpr121Error {
        InvalidDevice = 0,
        AlreadyOpen = 1,
        I2cNack = 2,
        I2cTimeout = 3,
        InvalidElectrodeCount = 4,
        OverCurrent = 5,
    }
}

// ── Device select ─────────────────────────────────────────────────────

mpr_enum! {
    #[derive(PartialEq, Eq)]
    pub enum Device { A = 0, B = 1 }
}

// ── Baseline filtering (registers 0x2B–0x35) ─────────────────────────

mpr_struct! {
    /// Baseline filter parameters for rising, falling, and touched
    /// scenarios. Controls how the MPR121 tracks slow background
    /// capacitance drift.
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
}

impl BaselineFilter {
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

mpr_struct! {
    /// Global touch/release thresholds applied to all enabled electrodes.
    /// Touch fires when baseline − filtered > touch_threshold.
    /// Release fires when baseline − filtered < release_threshold.
    /// Typical range: touch 4–16, release slightly less than touch.
    pub struct ThresholdConfig {
        pub touch: u8,
        pub release: u8,
    }
}

impl ThresholdConfig {
    pub const fn new(touch: u8, release: u8) -> Self {
        Self { touch, release }
    }
}

// ── Debounce (register 0x5B) ──────────────────────────────────────────

mpr_enum! {
    /// Consecutive detections required before a touch/release status
    /// change takes effect. Higher values reject noise at the cost
    /// of latency: delay = ESI × SFI × debounce.
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
}

mpr_struct! {
    pub struct DebounceConfig {
        pub touch: Debounce,
        pub release: Debounce,
    }
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

mpr_enum! {
    /// First filter iterations — number of ADC samples averaged in
    /// the first-level filter (register 0x5C bits [7:6]).
    pub enum FirstFilterIterations {
        Samples6 = 0,
        Samples10 = 1,
        Samples18 = 2,
        Samples34 = 3,
    }
}

mpr_enum! {
    /// Global charge/discharge time per measurement
    /// (register 0x5D bits [7:5]). Time = 2^(n−2) µs.
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
}

mpr_enum! {
    /// Second filter iterations — number of samples for the
    /// second-level filter (register 0x5D bits [4:3]).
    pub enum SecondFilterIterations {
        Samples4 = 0,
        Samples6 = 1,
        Samples10 = 2,
        Samples18 = 3,
    }
}

mpr_enum! {
    /// Electrode sample interval — period between second-level
    /// filter samples (register 0x5D bits [2:0]). Period = 2^n ms.
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
}

mpr_struct! {
    /// Analog front-end and digital filter configuration.
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

mpr_enum! {
    /// Controls baseline tracking and how the initial baseline
    /// value is loaded on entering run mode (ECR bits [7:6]).
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
}

mpr_enum! {
    /// Proximity detection mode — which electrodes are combined
    /// for the 13th proximity channel (ECR bits [5:4]).
    pub enum ProximityMode {
        Disabled = 0,
        Ele0To1 = 1,
        Ele0To3 = 2,
        Ele0To11 = 3,
    }
}

mpr_struct! {
    /// Electrode configuration register fields.
    pub struct ElectrodeConfig {
        /// Baseline tracking mode
        pub calibration: CalibrationLock,
        /// Proximity detection mode
        pub proximity: ProximityMode,
        /// Number of electrodes to enable (0–12)
        pub electrode_count: u8,
    }
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

mpr_enum! {
    /// Number of retries for auto-config before setting OOR
    /// (register 0x7B bits [3:2]).
    pub enum AutoConfigRetry {
        None = 0,
        Retry2 = 1,
        Retry4 = 2,
        Retry8 = 3,
    }
}

mpr_struct! {
    /// Auto-configuration settings. When enabled, the MPR121
    /// automatically searches for optimal CDC/CDT values per channel
    /// on each stop→run transition.
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
    /// USL = (VDD − 0.7) / VDD × 256, TL = USL × 0.9, LSL = USL × 0.65
    /// ARE (auto-reconfig) is disabled: disconnected/OOR electrodes are left
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

mpr_struct! {
    pub struct Mpr121Config {
        pub thresholds: ThresholdConfig,
        pub baseline: BaselineFilter,
        pub debounce: DebounceConfig,
        pub afe: AfeConfig,
        pub electrode: ElectrodeConfig,
        pub auto_config: AutoConfig,
    }
}

impl Mpr121Config {
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

// ── IPC trait ─────────────────────────────────────────────────────────

#[ipc::resource(arena_size = 2, kind = 0x0E)]
pub trait Mpr121 {
    #[constructor]
    fn open(device: Device, config: Mpr121Config) -> Result<Self, Mpr121Error>;

    /// Read 10-bit filtered electrode data for all 12 channels.
    #[message]
    fn read(&mut self) -> Result<[i32; 12], Mpr121Error>;

    /// Read touch status bitmask (bit N = electrode N touched).
    #[message]
    fn touch_status(&mut self) -> Result<u16, Mpr121Error>;

    /// Read 8-bit baseline values for all 12 channels.
    /// Left-shift by 2 to compare with 10-bit filtered data.
    #[message]
    fn read_baseline(&mut self) -> Result<[u8; 12], Mpr121Error>;

    /// Set per-electrode touch/release thresholds (overrides global).
    #[message]
    fn set_threshold(&mut self, electrode: u8, touch: u8, release: u8) -> Result<(), Mpr121Error>;

    /// Trigger a soft reset and reconfigure with the original config.
    #[message]
    fn reset(&mut self) -> Result<(), Mpr121Error>;
}
