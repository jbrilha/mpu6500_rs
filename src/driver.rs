use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::digital::Wait;
use embedded_hal_async::i2c::I2c;
use embedded_hal_async::spi::SpiDevice;

use crate::interface::{I2cInterface, Interface, SpiInterface};
use crate::registers::Register;
use crate::types::{AccelFs, GyroFs, Sample, SelfTestResult, Vec3};

pub enum Interrupt {
    MotionDetection,
    FIFOOverflow,
    DataReady,
}

impl Interrupt {
    const fn mask(self) -> u8 {
        match self {
            Interrupt::MotionDetection => 1 << 6,
            Interrupt::FIFOOverflow => 1 << 4,
            Interrupt::DataReady => 1 << 0,
        }
    }
}

#[derive(Debug)]
pub enum MPU6500Error<B, P> {
    Bus(B),
    Pin(P),
    NoDevice(u8),
}

const WHO_AM_I: u8 = 0x70;
const DLPF_41HZ: u8 = 0b011;
const FS_MASK: u8 = 0b11000;

pub struct MPU6500<B, INT, D> {
    bus: B,
    isr_pin: INT,
    delay: D,
    gyro_fs: GyroFs,
    accel_fs: AccelFs,
}

pub(crate) type Error<B, INT> =
    MPU6500Error<<B as Interface>::Error, <INT as embedded_hal::digital::ErrorType>::Error>;

impl<S: SpiDevice, INT: Wait, D: DelayNs> MPU6500<SpiInterface<S>, INT, D> {
    pub fn new_spi(spi: S, isr_pin: INT, delay: D) -> Self {
        Self::new(SpiInterface::new(spi), isr_pin, delay)
    }
}

impl<I: I2c, INT: Wait, D: DelayNs> MPU6500<I2cInterface<I>, INT, D> {
    pub fn new_i2c(i2c: I, address: u8, isr_pin: INT, delay: D) -> Self {
        Self::new(I2cInterface::new(i2c, address), isr_pin, delay)
    }
}

impl<B: Interface, INT: Wait, D: DelayNs> MPU6500<B, INT, D> {
    pub fn new(bus: B, isr_pin: INT, delay: D) -> Self {
        Self {
            bus,
            isr_pin,
            delay,
            gyro_fs: GyroFs::Dps250,
            accel_fs: AccelFs::G2,
        }
    }

    pub fn release(self) -> (B, INT, D) {
        (self.bus, self.isr_pin, self.delay)
    }

    pub(crate) async fn write_reg(
        &mut self,
        reg: Register,
        value: u8,
    ) -> Result<(), Error<B, INT>> {
        self.bus
            .write_reg(reg, value)
            .await
            .map_err(MPU6500Error::Bus)
    }

    pub(crate) async fn read_regs<const N: usize>(
        &mut self,
        reg: Register,
    ) -> Result<[u8; N], Error<B, INT>> {
        let mut buf = [0u8; N];
        self.bus
            .read_regs(reg, &mut buf)
            .await
            .map_err(MPU6500Error::Bus)?;
        Ok(buf)
    }

    pub(crate) async fn read_reg(&mut self, reg: Register) -> Result<u8, Error<B, INT>> {
        Ok(self.read_regs::<1>(reg).await?[0])
    }

    async fn modify_reg(
        &mut self,
        reg: Register,
        mask: u8,
        value: u8,
    ) -> Result<(), Error<B, INT>> {
        let current = self.read_reg(reg).await?;
        self.write_reg(reg, (current & !mask) | (value & mask))
            .await
    }

    pub(crate) async fn delay_ms(&mut self, ms: u32) {
        self.delay.delay_ms(ms).await;
    }

    pub async fn enable_interrupt(&mut self, interrupt: Interrupt) -> Result<(), Error<B, INT>> {
        let mask = interrupt.mask();
        self.modify_reg(Register::IntEnable, mask, mask).await
    }

    pub async fn disable_interrupt(&mut self, interrupt: Interrupt) -> Result<(), Error<B, INT>> {
        let mask = interrupt.mask();
        self.modify_reg(Register::IntEnable, mask, 0).await
    }

    pub async fn init(&mut self) -> Result<(), Error<B, INT>> {
        // reset then wait for the internal registers to settle
        self.write_reg(Register::PwrMgmt1, 0b10000000).await?;
        self.delay_ms(100).await;

        // auto-select the best clock source, wake every axis
        self.write_reg(Register::PwrMgmt1, 0b001).await?;
        self.write_reg(Register::PwrMgmt2, 0b000000).await?;
        self.delay_ms(10).await;

        if B::DISABLE_I2C {
            self.write_reg(Register::UserCtrl, 0b10000).await?;
        }

        let id = self.read_reg(Register::WhoAmI).await?;
        if id == 0x00 || id == 0xFF {
            return Err(MPU6500Error::NoDevice(id));
        }
        if id != WHO_AM_I {
            warn!(
                "unexpected WHO_AM_I {=u8:#04x}, expected {=u8:#04x}",
                id, WHO_AM_I
            );
        }

        // 1kHz internal sample rate, no division
        self.write_reg(Register::SmplrtDiv, 0b00).await?;
        self.write_reg(Register::Config, DLPF_41HZ).await?;
        self.write_reg(Register::AccelConfig2, DLPF_41HZ).await?;

        let gyro_fs = self.gyro_fs;
        let accel_fs = self.accel_fs;
        self.set_gyro_fs(gyro_fs).await?;
        self.set_accel_fs(accel_fs).await?;

        // active high, push-pull
        self.write_reg(Register::IntPinCfg, 0b00000000).await?;
        self.enable_interrupt(Interrupt::DataReady).await?;

        info!("mpu6500 init done, id {=u8:#04x}", id);
        Ok(())
    }

    pub async fn read_gyro(&mut self) -> Result<Vec3<i16>, Error<B, INT>> {
        let raw = self.read_regs::<6>(Register::GyroXOutH).await?;
        Ok(Vec3::from_be_bytes(raw))
    }

    pub async fn read_accel(&mut self) -> Result<Vec3<i16>, Error<B, INT>> {
        let raw = self.read_regs::<6>(Register::AccelXOutH).await?;
        Ok(Vec3::from_be_bytes(raw))
    }

    /// burst over the contiguous output block
    pub async fn read_all(&mut self) -> Result<Sample, Error<B, INT>> {
        // ACCEL_XOUT_H through GYRO_ZOUT_L are contiguous, temperature included
        let raw = self.read_regs::<14>(Register::AccelXOutH).await?;
        Ok(Sample {
            accel: Vec3::from_be_bytes([raw[0], raw[1], raw[2], raw[3], raw[4], raw[5]]),
            temp: i16::from_be_bytes([raw[6], raw[7]]),
            gyro: Vec3::from_be_bytes([raw[8], raw[9], raw[10], raw[11], raw[12], raw[13]]),
        })
    }

    pub async fn self_test(&mut self) -> Result<SelfTestResult, Error<B, INT>> {
        self.run_self_test().await
    }

    pub async fn set_sleep(&mut self) -> Result<(), Error<B, INT>> {
        self.write_reg(Register::PwrMgmt1, 0b01000000).await
    }

    pub async fn set_gyro_standby(&mut self) -> Result<(), Error<B, INT>> {
        self.write_reg(Register::PwrMgmt1, 0b10000).await
    }

    pub async fn set_gyro_fs(&mut self, fs: GyroFs) -> Result<(), Error<B, INT>> {
        self.modify_reg(Register::GyroConfig, FS_MASK, fs.bits())
            .await?;
        self.gyro_fs = fs;
        Ok(())
    }

    pub async fn set_accel_fs(&mut self, fs: AccelFs) -> Result<(), Error<B, INT>> {
        self.modify_reg(Register::AccelConfig, FS_MASK, fs.bits())
            .await?;
        self.accel_fs = fs;
        Ok(())
    }

    pub fn gyro_fs(&self) -> GyroFs {
        self.gyro_fs
    }

    pub fn accel_fs(&self) -> AccelFs {
        self.accel_fs
    }

    pub async fn wait_for_isr(&mut self) -> Result<(), Error<B, INT>> {
        self.isr_pin
            .wait_for_rising_edge()
            .await
            .map_err(MPU6500Error::Pin)
    }
}
