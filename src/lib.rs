#![no_std]

#[macro_use]
mod fmt;

mod driver;
mod interface;
pub mod registers;
mod self_test;
mod types;

pub use driver::{Interrupt, MPU6500, MPU6500Error};
pub use interface::{ADDRESS_AD0_HIGH, ADDRESS_AD0_LOW, I2cInterface, Interface, SpiInterface};
pub use types::{AccelFs, GyroFs, Sample, SelfTestResult, Vec3};
