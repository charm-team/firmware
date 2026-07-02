#![no_std]
#![no_main]

use generated::slots::SLOTS;
use rcard_log::{error, info};
use sysmodule_mpr121_api::*;

sysmodule_log_api::bind_log!(Log = SLOTS.sysmodule_log);
rcard_log::bind_logger!(Log);
sysmodule_log_api::panic_handler!(Log);
sysmodule_clocks_api::bind_clocks!(Clocks = SLOTS.sysmodule_clocks);
sysmodule_power_api::bind_power!(Power = SLOTS.sysmodule_power);

// Both MPR121s use default address 0x5A (ADDR pin to VSS), on separate buses.
// Device::A = I2C2 (PA32 SDA, PA33 SCL)
// Device::B = I2C3 (PA30 SDA, PA31 SCL)
const I2C_ADDR: u8 = 0x5A;

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

const I2C_TIMEOUT_MS: u64 = 50;

// ── I2C low-level (same hardware as CAP1208) ─────────────────────────

fn ticks_now() -> u64 {
    userlib::sys_get_timer().now
}

fn i2c_for(device: Device) -> sifli_pac::i2c::I2c {
    match device {
        Device::A => sifli_pac::I2C2,
        Device::B => sifli_pac::I2C3,
    }
}

fn init_i2c_bus(i2c: sifli_pac::i2c::I2c) {
    // fclk = 48 MHz, 400 kHz fast mode
    // flv = ((48M + 200k) / 400k - 7 + 1) / 2 = 57
    i2c.cr().write(|w| w.set_ur(true));
    for _ in 0..50u32 {
        core::hint::spin_loop();
    }
    i2c.cr().write(|w| {
        w.set_mode(0x01);
        w.set_scle(true);
    });
    i2c.lcr().write(|w| {
        w.set_flv(57);
        w.set_slv(0x1FF);
    });
    i2c.ier().write(|_| {});
    i2c.wcr().write(|w| w.set_cnt(25));
}

fn i2c_enable(i2c: sifli_pac::i2c::I2c) {
    i2c.cr().modify(|w| w.set_iue(true));
}

fn i2c_disable(i2c: sifli_pac::i2c::I2c) {
    i2c.cr().modify(|w| w.set_iue(false));
}

fn wait_te(i2c: sifli_pac::i2c::I2c) -> bool {
    let deadline = ticks_now() + I2C_TIMEOUT_MS;
    loop {
        let sr = i2c.sr().read();
        if sr.te() {
            i2c.sr().write(|w| w.set_te(true));
            return true;
        }
        if sr.bed() {
            i2c.sr().write(|w| w.set_bed(true));
            error!("i2c bus error during transmit");
            return false;
        }
        if ticks_now() >= deadline {
            let bmr = i2c.bmr().read();
            error!(
                "i2c TE timeout: sr={} scl={} sda={}",
                sr.0,
                bmr.scl() as u8,
                bmr.sda() as u8,
            );
            return false;
        }
    }
}

fn wait_rf(i2c: sifli_pac::i2c::I2c) -> bool {
    let deadline = ticks_now() + I2C_TIMEOUT_MS;
    loop {
        let sr = i2c.sr().read();
        if sr.rf() {
            i2c.sr().write(|w| w.set_rf(true));
            return true;
        }
        if sr.bed() {
            i2c.sr().write(|w| w.set_bed(true));
            error!("i2c bus error during receive");
            return false;
        }
        if ticks_now() >= deadline {
            let bmr = i2c.bmr().read();
            error!(
                "i2c RF timeout: sr={} scl={} sda={}",
                sr.0,
                bmr.scl() as u8,
                bmr.sda() as u8,
            );
            return false;
        }
    }
}

fn i2c_write_reg(i2c: sifli_pac::i2c::I2c, reg: u8, val: u8) -> Result<(), Mpr121Error> {
    i2c_enable(i2c);

    i2c.dbr().write(|w| w.set_data(I2C_ADDR << 1));
    i2c.tcr().write(|w| {
        w.set_start(true);
        w.set_tb(true);
    });
    if !wait_te(i2c) {
        i2c_disable(i2c);
        return Err(Mpr121Error::I2cTimeout);
    }
    if i2c.sr().read().nack() {
        i2c.tcr().write(|w| w.set_ma(true));
        i2c_disable(i2c);
        return Err(Mpr121Error::I2cNack);
    }

    i2c.dbr().write(|w| w.set_data(reg));
    i2c.tcr().write(|w| w.set_tb(true));
    if !wait_te(i2c) {
        i2c_disable(i2c);
        return Err(Mpr121Error::I2cTimeout);
    }
    if i2c.sr().read().nack() {
        i2c.tcr().write(|w| w.set_ma(true));
        i2c_disable(i2c);
        return Err(Mpr121Error::I2cNack);
    }

    i2c.dbr().write(|w| w.set_data(val));
    i2c.tcr().write(|w| {
        w.set_stop(true);
        w.set_tb(true);
    });
    if !wait_te(i2c) {
        i2c_disable(i2c);
        return Err(Mpr121Error::I2cTimeout);
    }

    i2c_disable(i2c);
    Ok(())
}

fn i2c_read_reg(i2c: sifli_pac::i2c::I2c, reg: u8) -> Result<u8, Mpr121Error> {
    i2c_enable(i2c);

    i2c.dbr().write(|w| w.set_data(I2C_ADDR << 1));
    i2c.tcr().write(|w| {
        w.set_start(true);
        w.set_tb(true);
    });
    if !wait_te(i2c) {
        i2c_disable(i2c);
        return Err(Mpr121Error::I2cTimeout);
    }
    if i2c.sr().read().nack() {
        i2c.tcr().write(|w| w.set_ma(true));
        i2c_disable(i2c);
        return Err(Mpr121Error::I2cNack);
    }

    i2c.dbr().write(|w| w.set_data(reg));
    i2c.tcr().write(|w| w.set_tb(true));
    if !wait_te(i2c) {
        i2c_disable(i2c);
        return Err(Mpr121Error::I2cTimeout);
    }
    if i2c.sr().read().nack() {
        i2c.tcr().write(|w| w.set_ma(true));
        i2c_disable(i2c);
        return Err(Mpr121Error::I2cNack);
    }

    i2c.dbr().write(|w| w.set_data((I2C_ADDR << 1) | 1));
    i2c.tcr().write(|w| {
        w.set_start(true);
        w.set_tb(true);
    });
    if !wait_te(i2c) {
        i2c_disable(i2c);
        return Err(Mpr121Error::I2cTimeout);
    }
    if i2c.sr().read().nack() {
        i2c.tcr().write(|w| w.set_ma(true));
        i2c_disable(i2c);
        return Err(Mpr121Error::I2cNack);
    }

    i2c.tcr().write(|w| {
        w.set_nack(true);
        w.set_stop(true);
        w.set_tb(true);
    });
    if !wait_rf(i2c) {
        i2c_disable(i2c);
        return Err(Mpr121Error::I2cTimeout);
    }

    let val = i2c.dbr().read().data();
    i2c_disable(i2c);
    Ok(val)
}

fn i2c_read_block(
    i2c: sifli_pac::i2c::I2c,
    start_reg: u8,
    buf: &mut [u8],
) -> Result<(), Mpr121Error> {
    if buf.is_empty() {
        return Ok(());
    }

    i2c_enable(i2c);

    i2c.dbr().write(|w| w.set_data(I2C_ADDR << 1));
    i2c.tcr().write(|w| {
        w.set_start(true);
        w.set_tb(true);
    });
    if !wait_te(i2c) {
        i2c_disable(i2c);
        return Err(Mpr121Error::I2cTimeout);
    }
    if i2c.sr().read().nack() {
        i2c.tcr().write(|w| w.set_ma(true));
        i2c_disable(i2c);
        return Err(Mpr121Error::I2cNack);
    }

    i2c.dbr().write(|w| w.set_data(start_reg));
    i2c.tcr().write(|w| w.set_tb(true));
    if !wait_te(i2c) {
        i2c_disable(i2c);
        return Err(Mpr121Error::I2cTimeout);
    }
    if i2c.sr().read().nack() {
        i2c.tcr().write(|w| w.set_ma(true));
        i2c_disable(i2c);
        return Err(Mpr121Error::I2cNack);
    }

    i2c.dbr().write(|w| w.set_data((I2C_ADDR << 1) | 1));
    i2c.tcr().write(|w| {
        w.set_start(true);
        w.set_tb(true);
    });
    if !wait_te(i2c) {
        i2c_disable(i2c);
        return Err(Mpr121Error::I2cTimeout);
    }
    if i2c.sr().read().nack() {
        i2c.tcr().write(|w| w.set_ma(true));
        i2c_disable(i2c);
        return Err(Mpr121Error::I2cNack);
    }

    let last = buf.len() - 1;
    let mut i = 0;
    while i <= last {
        if i == last {
            i2c.tcr().write(|w| {
                w.set_nack(true);
                w.set_stop(true);
                w.set_tb(true);
            });
        } else {
            i2c.tcr().write(|w| w.set_tb(true));
        }
        if !wait_rf(i2c) {
            i2c_disable(i2c);
            return Err(Mpr121Error::I2cTimeout);
        }
        buf[i] = i2c.dbr().read().data();
        i += 1;
    }

    i2c_disable(i2c);
    Ok(())
}

// ── MPR121 chip configuration ─────────────────────────────────────────

fn soft_reset(i2c: sifli_pac::i2c::I2c) -> Result<(), Mpr121Error> {
    i2c_write_reg(i2c, REG_SOFT_RESET, 0x63)
}

fn configure_chip(i2c: sifli_pac::i2c::I2c, config: &Mpr121Config) -> Result<(), Mpr121Error> {
    let ecfg = &config.electrode;
    if ecfg.electrode_count > 12 {
        return Err(Mpr121Error::InvalidElectrodeCount);
    }

    soft_reset(i2c)?;

    // After reset, ECR defaults to 0x00 (stop mode). All configuration
    // must happen in stop mode.

    let bl = &config.baseline;
    i2c_write_reg(i2c, REG_MHD_RISING, bl.mhd_rising)?;
    i2c_write_reg(i2c, REG_NHD_RISING, bl.nhd_rising)?;
    i2c_write_reg(i2c, REG_NCL_RISING, bl.ncl_rising)?;
    i2c_write_reg(i2c, REG_FDL_RISING, bl.fdl_rising)?;
    i2c_write_reg(i2c, REG_MHD_FALLING, bl.mhd_falling)?;
    i2c_write_reg(i2c, REG_NHD_FALLING, bl.nhd_falling)?;
    i2c_write_reg(i2c, REG_NCL_FALLING, bl.ncl_falling)?;
    i2c_write_reg(i2c, REG_FDL_FALLING, bl.fdl_falling)?;
    i2c_write_reg(i2c, REG_NHD_TOUCHED, bl.nhd_touched)?;
    i2c_write_reg(i2c, REG_NCL_TOUCHED, bl.ncl_touched)?;
    i2c_write_reg(i2c, REG_FDL_TOUCHED, bl.fdl_touched)?;

    let th = &config.thresholds;
    for ele in 0..ecfg.electrode_count {
        let base = REG_TOUCH_THRESHOLD_BASE + (ele * 2);
        i2c_write_reg(i2c, base, th.touch)?;
        i2c_write_reg(i2c, base + 1, th.release)?;
    }

    let db = &config.debounce;
    i2c_write_reg(
        i2c,
        REG_DEBOUNCE,
        ((db.release as u8) << 4) | (db.touch as u8),
    )?;

    let afe = &config.afe;
    // 0x5C: FFI [7:6] | CDC [5:0]
    i2c_write_reg(
        i2c,
        REG_AFE_CONFIG1,
        ((afe.ffi as u8) << 6) | (afe.cdc & 0x3F),
    )?;
    // 0x5D: CDT [7:5] | SFI [4:3] | ESI [2:0]
    i2c_write_reg(
        i2c,
        REG_AFE_CONFIG2,
        ((afe.cdt as u8) << 5) | ((afe.sfi as u8) << 3) | (afe.esi as u8),
    )?;

    let ac = &config.auto_config;
    if ac.enabled != 0 {
        // 0x7B: FFI [7:6] | RETRY [3:2] | BVA [1:0] (BVA mirrors CL) | ARE [1] | ACE [0]
        // Actually: FFI[7:6] RETRY[5:4] BVA[3:2] ARE[1] ACE[0]
        let ac0 = ((afe.ffi as u8) << 6)
            | ((ac.retry as u8) << 4)
            | ((ecfg.calibration as u8) << 2)
            | ((ac.reconfig_enabled & 1) << 1)
            | (ac.enabled & 1);
        i2c_write_reg(i2c, REG_AUTOCONFIG0, ac0)?;
        // 0x7C: SCTS [7]
        i2c_write_reg(i2c, REG_AUTOCONFIG1, (ac.skip_charge_time & 1) << 7)?;
        i2c_write_reg(i2c, REG_AUTOCONFIG_USL, ac.upper_limit)?;
        i2c_write_reg(i2c, REG_AUTOCONFIG_LSL, ac.lower_limit)?;
        i2c_write_reg(i2c, REG_AUTOCONFIG_TL, ac.target_level)?;
    }

    // ECR: CL [7:6] | ELEPROX_EN [5:4] | ELE_EN [3:0]
    let ecr = ((ecfg.calibration as u8) << 6)
        | ((ecfg.proximity as u8) << 4)
        | (ecfg.electrode_count & 0x0F);
    i2c_write_reg(i2c, REG_ECR, ecr)?;

    Ok(())
}

// ── IPC resource ──────────────────────────────────────────────────────

struct Mpr121Resource {
    device: Device,
    config: Mpr121Config,
}

static mut SLOT_USED: [bool; 2] = [false; 2];

impl Mpr121 for Mpr121Resource {
    fn open(meta: ipc::Meta, device: Device, config: Mpr121Config) -> Result<Self, Mpr121Error> {
        let _ = meta;
        let idx = device as usize;

        if unsafe { SLOT_USED[idx] } {
            error!("device {} already open", device);
            return Err(Mpr121Error::AlreadyOpen);
        }

        let i2c = i2c_for(device);

        info!("opening device {}, enabling LDO2", device);
        match Power::enable_ldo(sysmodule_power_api::Ldo::Vdd33Ldo2) {
            Ok(Ok(())) => info!("LDO2 enabled"),
            Ok(Err(e)) => error!("LDO2 failed: {}", e),
            Err(e) => error!("LDO2 IPC failed: {}", e),
        }

        let deadline = userlib::sys_get_timer().now + 10;
        while userlib::sys_get_timer().now < deadline {}

        configure_chip(i2c, &config)?;
        info!(
            "device {}: configured, {} electrodes",
            device, config.electrode.electrode_count
        );

        unsafe { SLOT_USED[idx] = true };

        Ok(Mpr121Resource { device, config })
    }

    fn read(&mut self, _meta: ipc::Meta) -> Result<[i32; 12], Mpr121Error> {
        let i2c = i2c_for(self.device);
        let count = self.config.electrode.electrode_count as usize;
        let mut raw = [0u8; 24];
        i2c_read_block(i2c, REG_FILTERED_DATA_BASE, &mut raw[..count * 2])?;

        let mut result = [0i32; 12];
        let mut i = 0;
        while i < count {
            let lo = raw[i * 2] as u16;
            let hi = raw[i * 2 + 1] as u16;
            result[i] = ((hi << 8) | lo) as i32;
            i += 1;
        }

        Ok(result)
    }

    fn touch_status(&mut self, _meta: ipc::Meta) -> Result<u16, Mpr121Error> {
        let i2c = i2c_for(self.device);
        let mut raw = [0u8; 2];
        i2c_read_block(i2c, REG_TOUCH_STATUS_L, &mut raw)?;
        let status = (raw[1] as u16) << 8 | (raw[0] as u16);
        if status & (1 << 15) != 0 {
            return Err(Mpr121Error::OverCurrent);
        }
        Ok(status & 0x0FFF)
    }

    fn read_baseline(&mut self, _meta: ipc::Meta) -> Result<[u8; 12], Mpr121Error> {
        let i2c = i2c_for(self.device);
        let count = self.config.electrode.electrode_count as usize;
        let mut raw = [0u8; 12];
        i2c_read_block(i2c, REG_BASELINE_BASE, &mut raw[..count])?;
        Ok(raw)
    }

    fn set_threshold(
        &mut self,
        _meta: ipc::Meta,
        electrode: u8,
        touch: u8,
        release: u8,
    ) -> Result<(), Mpr121Error> {
        if electrode >= self.config.electrode.electrode_count {
            return Err(Mpr121Error::InvalidElectrodeCount);
        }
        let i2c = i2c_for(self.device);
        // Must be in stop mode to write config registers
        i2c_write_reg(i2c, REG_ECR, 0x00)?;
        let base = REG_TOUCH_THRESHOLD_BASE + (electrode * 2);
        i2c_write_reg(i2c, base, touch)?;
        i2c_write_reg(i2c, base + 1, release)?;
        // Re-enter run mode
        let ecfg = &self.config.electrode;
        let ecr = ((ecfg.calibration as u8) << 6)
            | ((ecfg.proximity as u8) << 4)
            | (ecfg.electrode_count & 0x0F);
        i2c_write_reg(i2c, REG_ECR, ecr)?;
        Ok(())
    }

    fn reset(&mut self, _meta: ipc::Meta) -> Result<(), Mpr121Error> {
        let i2c = i2c_for(self.device);
        info!("device {}: reset", self.device);
        configure_chip(i2c, &self.config)
    }
}

impl Drop for Mpr121Resource {
    fn drop(&mut self) {
        unsafe { SLOT_USED[self.device as usize] = false };
        // Put device back into stop mode
        let _ = i2c_write_reg(i2c_for(self.device), REG_ECR, 0x00);
    }
}

// ── Entry point ───────────────────────────────────────────────────────

#[export_name = "main"]
fn main() -> ! {
    info!("mpr121: starting");

    match Power::enable_ldo(sysmodule_power_api::Ldo::Vdd33Ldo2) {
        Ok(Ok(())) => info!("mpr121: VDD33_LDO2 enabled"),
        Ok(Err(e)) => error!("mpr121: failed to enable LDO2: {}", e),
        Err(e) => error!("mpr121: LDO2 IPC failed: {}", e),
    }

    if let Err(e) = Clocks::enable(sysmodule_clocks_api::Peripheral::I2c2) {
        error!("failed to enable I2C2 clock: {}", e);
    }
    if let Err(e) = Clocks::enable(sysmodule_clocks_api::Peripheral::I2c3) {
        error!("failed to enable I2C3 clock: {}", e);
    }
    if let Err(e) = Clocks::reset(sysmodule_clocks_api::Peripheral::I2c2) {
        error!("failed to reset I2C2: {}", e);
    }
    if let Err(e) = Clocks::reset(sysmodule_clocks_api::Peripheral::I2c3) {
        error!("failed to reset I2C3: {}", e);
    }

    let cfg = sifli_pac::HPSYS_CFG;
    cfg.i2c2_pinr().write(|w| {
        w.set_scl_pin(32); // PA32
        w.set_sda_pin(33); // PA33
    });
    cfg.i2c3_pinr().write(|w| {
        w.set_scl_pin(30); // PA30
        w.set_sda_pin(31); // PA31
    });

    init_i2c_bus(sifli_pac::I2C2);
    init_i2c_bus(sifli_pac::I2C3);

    info!("mpr121: i2c buses initialized, entering server loop");

    ipc::server! {
        Mpr121: Mpr121Resource,
    }
}
