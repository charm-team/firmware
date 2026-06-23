//! Compile-time framebuffers built from 128x64 PNGs in `bootloader/images/`.
//!
//! Drop a PNG at `bootloader/images/<name>.png` and it appears here as
//! `pub static <NAME>: FrameBuffer`, where `<NAME>` is the file stem
//! upper-cased with non-alphanumerics replaced by `_`. The packing matches
//! [`crate::oled::FrameBuffer`] (page-major 1bpp, LSB = top of page).
//!
//! Pixel threshold: `(luma * alpha) / 255 >= 128`, where luma uses ITU-R BT.601
//! weights. Transparent regions count as off.

include!(concat!(env!("OUT_DIR"), "/images.rs"));
