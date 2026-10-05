macro_rules! info {
    ($($arg:tt)*) => {
        #[cfg(feature = "defmt")]
        ::defmt::info!($($arg)*);
    };
}

macro_rules! warn {
    ($($arg:tt)*) => {
        #[cfg(feature = "defmt")]
        ::defmt::warn!($($arg)*);
    };
}

macro_rules! error {
    ($($arg:tt)*) => {
        #[cfg(feature = "defmt")]
        ::defmt::error!($($arg)*);
    };
}
