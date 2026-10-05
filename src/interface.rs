use embedded_hal_async::i2c::I2c;
use embedded_hal_async::spi::{Operation, SpiDevice};

use crate::registers::Register;

pub const ADDRESS_AD0_LOW: u8 = 0x68;
pub const ADDRESS_AD0_HIGH: u8 = 0x69;

pub trait Interface {
    type Error;
    const DISABLE_I2C: bool;

    #[allow(async_fn_in_trait)]
    async fn write_reg(&mut self, reg: Register, value: u8) -> Result<(), Self::Error>;
    #[allow(async_fn_in_trait)]
    async fn read_regs(&mut self, reg: Register, buf: &mut [u8]) -> Result<(), Self::Error>;
}

pub struct SpiInterface<S> {
    spi: S,
}

impl<S: SpiDevice> SpiInterface<S> {
    pub fn new(spi: S) -> Self {
        Self { spi }
    }

    pub fn release(self) -> S {
        self.spi
    }
}

impl<S: SpiDevice> Interface for SpiInterface<S> {
    type Error = S::Error;
    const DISABLE_I2C: bool = true;

    async fn write_reg(&mut self, reg: Register, value: u8) -> Result<(), Self::Error> {
        self.spi
            .transaction(&mut [Operation::Write(&[reg.addr(), value])])
            .await
    }

    async fn read_regs(&mut self, reg: Register, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.spi
            .transaction(&mut [Operation::Write(&[reg.read_addr()]), Operation::Read(buf)])
            .await
    }
}

pub struct I2cInterface<I> {
    i2c: I,
    address: u8,
}

impl<I: I2c> I2cInterface<I> {
    pub fn new(i2c: I, address: u8) -> Self {
        Self { i2c, address }
    }

    pub fn release(self) -> I {
        self.i2c
    }
}

impl<I: I2c> Interface for I2cInterface<I> {
    type Error = I::Error;
    const DISABLE_I2C: bool = false;

    async fn write_reg(&mut self, reg: Register, value: u8) -> Result<(), Self::Error> {
        self.i2c.write(self.address, &[reg.addr(), value]).await
    }

    async fn read_regs(&mut self, reg: Register, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.write_read(self.address, &[reg.addr()], buf).await
    }
}
