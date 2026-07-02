fn usart() -> sifli_pac::usart::Usart {
    sifli_pac::USART1
}

pub fn usart_init() {
    let u = usart();
    // BRR = 48MHz / 1000000 = 48 (0x30)
    u.brr().write(|w| w.0 = 0x30);
    // CR1: UE | TE
    u.cr1().write(|w| {
        w.set_m(sifli_pac::usart::vals::M::Bit8);
        w.set_ue(true);
        w.set_te(true);
    });
}

pub fn usart_write_bytes(msg: &[u8]) {
    let u = usart();
    // Acquire EXR lock: reading busy==0 atomically sets it to 1.
    while u.exr().read().busy() {}
    for &b in msg {
        while !u.isr().read().txe() {}
        u.tdr().write(|w| w.0 = b as u32);
    }
    // Wait for transmission to fully complete before releasing.
    while !u.isr().read().tc() {}
    // Release EXR lock: write 1 to busy to unlock.
    u.exr().write(|w| w.set_busy(true));
}

#[inline(always)]
pub fn usart_write_str(msg: &str) {
    usart_write_bytes(msg.as_bytes());
}

pub fn usart_write_u32(mut val: u32) {
    if val == 0 {
        usart_write_bytes(b"0");
        return;
    }
    static mut BUF: [u8; 10] = [0; 10];
    let buf = unsafe { &mut *(&raw mut BUF) };
    let mut i = buf.len();
    while val > 0 && i > 0 {
        i -= 1;
        // SAFETY: i is in 0..buf.len() because we check i > 0 above.
        unsafe { *buf.get_unchecked_mut(i) = b'0' + (val % 10) as u8 };
        val /= 10;
    }
    usart_write_bytes(unsafe { buf.get_unchecked(i..) });
}

pub fn usart_write_hex(mut val: u32) {
    usart_write_bytes(b"0x");
    if val == 0 {
        usart_write_bytes(b"0");
        return;
    }
    static mut BUF: [u8; 8] = [0; 8];
    let buf = unsafe { &mut *(&raw mut BUF) };
    let mut i = buf.len();
    while val > 0 && i > 0 {
        i -= 1;
        let nib = (val & 0xF) as u8;
        // SAFETY: i is in 0..buf.len() because we check i > 0 above.
        unsafe {
            *buf.get_unchecked_mut(i) = if nib < 10 {
                b'0' + nib
            } else {
                b'a' + nib - 10
            };
        }
        val >>= 4;
    }
    usart_write_bytes(unsafe { buf.get_unchecked(i..) });
}
