use core::{
    cell::{Cell, OnceCell, RefCell},
    fmt::Write,
};

use critical_section::Mutex;
use log::{LevelFilter, Metadata, Record};
use sifli_hal::{mode::Blocking, peripherals::USART1, usart::Uart};

struct UsartLogger;

type Usart = Uart<'static, USART1, Blocking>;

static DEBUG_USART: Mutex<RefCell<Option<Usart>>> = Mutex::new(RefCell::new(None));
static LOGGER: UsartLogger = UsartLogger;

pub fn init_logger(uart: Usart, level: LevelFilter) {
    critical_section::with(|cs| {
        DEBUG_USART.borrow_ref_mut(cs).replace(uart);
    });
    log::set_logger(&LOGGER).unwrap();
    log::set_max_level(level);
}

// bridges core::fmt::Write -> blocking_write
struct Adapter<'a>(&'a mut Usart);

impl Write for Adapter<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.0
            .blocking_write(s.as_bytes())
            .map_err(|_| core::fmt::Error)
    }
}

impl log::Log for UsartLogger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        true
    }

    fn log(&self, record: &Record) {
        if !self.enabled(record.metadata()) {
            return;
        }
        critical_section::with(|cs| {
            let mut guard = DEBUG_USART.borrow_ref_mut(cs);
            if let Some(uart) = guard.as_mut() {
                let _ = writeln!(
                    Adapter(uart),
                    "[{}] {}: {}",
                    record.level(),
                    record.target(),
                    record.args()
                );
            }
        });
    }

    fn flush(&self) {}
}
