use core::array::from_fn;

use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::digital::Wait;
use micromath::F32Ext;

use crate::driver::{Error, MPU6500};
use crate::interface::Interface;
use crate::registers::Register;
use crate::types::{SelfTestResult, Vec3};

const ST_OTP_BASE: f32 = 2620.0; // 2620 / 2^FS, with FS = 0 per AN-MPU-9250A-03 §3
const FS0_DPS_OFFSET_THRESH: i32 = 20 * 131; // table 1 from the datasheet
const OTP0_DPS_OFFSET_THRESH: i32 = 60 * 131; // table 1 from the datasheet
const OTP0_MGEE_OFFSET_MIN_THRESH: i32 = (225 * 16384) / 1000; // table 2 from the datasheet
const OTP0_MGEE_OFFSET_MAX_THRESH: i32 = (675 * 16384) / 1000; // table 2 from the datasheet

const SAMPLES: i32 = 200;

fn st_otp(code: u8) -> f32 {
    if code == 0 {
        0.0
    } else {
        ST_OTP_BASE * 1.01f32.powi(code as i32 - 1)
    }
}

impl<B: Interface, INT: Wait, D: DelayNs> MPU6500<B, INT, D> {
    async fn average(&mut self) -> Result<([i32; 3], [i32; 3]), Error<B, INT>> {
        let mut accel_avg = [0i32; 3];
        let mut gyro_avg = [0i32; 3];

        for _ in 0..SAMPLES {
            let sample = self.read_all().await?;

            accel_avg[0] += sample.accel.x as i32;
            accel_avg[1] += sample.accel.y as i32;
            accel_avg[2] += sample.accel.z as i32;

            gyro_avg[0] += sample.gyro.x as i32;
            gyro_avg[1] += sample.gyro.y as i32;
            gyro_avg[2] += sample.gyro.z as i32;

            self.wait_for_isr().await?;
        }

        for i in 0..3 {
            accel_avg[i] /= SAMPLES;
            gyro_avg[i] /= SAMPLES;
        }

        Ok((accel_avg, gyro_avg))
    }

    // from the AN-MPU-9250A-03 doc because I couldn't find the one specifically for the 6500...
    pub(crate) async fn run_self_test(&mut self) -> Result<SelfTestResult, Error<B, INT>> {
        // 0. set sample rate to 1kHz
        // SAMPLE_RATE = SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
        //  where INTERNAL_SAMPLE_RATE = 1kHz
        self.write_reg(Register::SmplrtDiv, 0b00).await?;

        // also reset the int pin config and enable the interrupt on raw values ready to ensure 200
        // reads later
        let old_int_enable = self.read_reg(Register::IntEnable).await?;
        self.write_reg(Register::IntPinCfg, 0b00000000).await?;
        self.write_reg(Register::IntEnable, 0b01).await?;

        // ideally both of these combined should make it so there's 200 reads at 1kHz but idk TODO
        // maybe

        // 1. (gyro)
        // set DLPF (1Ah — 2:0) code to 2
        self.write_reg(Register::Config, 0b010).await?;
        // store full scale range select code (1Bh — 4:3)
        let gyro_old_fs = self.read_reg(Register::GyroConfig).await?;
        // select full scale range
        self.write_reg(Register::GyroConfig, 0b00000).await?;

        // 1 (accel). set DLPF (1Dh — 2:0) code to 2
        self.write_reg(Register::AccelConfig2, 0b0010).await?;
        // store full scale range select code (1Ch — 4:3)
        let accel_old_fs = self.read_reg(Register::AccelConfig).await?;
        // select full scale range
        self.write_reg(Register::AccelConfig, 0b00000).await?;

        // 2. read the gyro and accel output at 1kHz and average 200 readings
        let (accel_avg, gyro_avg) = self.average().await?;

        // 3. enable self test on all axis (axes? axi?)
        self.write_reg(Register::GyroConfig, 0b11100000).await?;
        self.write_reg(Register::AccelConfig, 0b11100000).await?;

        // 4. wait for oscillations to stabilize
        self.delay_ms(20).await;

        // 5. read and average 200
        let (accel_st_avg, gyro_st_avg) = self.average().await?;

        // 6. calculate st response
        let gyro_st_resp: [i32; 3] = from_fn(|i| gyro_st_avg[i] - gyro_avg[i]);
        let accel_st_resp: [i32; 3] = from_fn(|i| accel_st_avg[i] - accel_avg[i]);

        // clean up after
        // 0. reset the int enable
        self.write_reg(Register::IntEnable, old_int_enable).await?;
        // 1. reset the axes
        self.write_reg(Register::GyroConfig, 0b00).await?;
        self.write_reg(Register::AccelConfig, 0b00).await?;

        // 2. yawn
        self.delay_ms(20).await;

        // 3. restore old_fs's
        self.write_reg(Register::GyroConfig, gyro_old_fs).await?;
        self.write_reg(Register::AccelConfig, accel_old_fs).await?;

        // pass/fail criteria
        // 1. get st_codes
        let gyro_st_code = self.read_regs::<3>(Register::SelfTestXGyro).await?;
        let accel_st_code = self.read_regs::<3>(Register::SelfTestXAccel).await?;

        // 2. calculate st_otp
        let gyro_st_otp = gyro_st_code.map(st_otp);
        let accel_st_otp = accel_st_code.map(st_otp);

        // determine pass or fail
        let mut failed = false;
        let mut gyro_st_res = [0f32; 3];
        let mut accel_st_res = [0f32; 3];
        for i in 0..3 {
            let g = if gyro_st_otp[i] != 0.0 {
                let g = gyro_st_resp[i] as f32 / gyro_st_otp[i];
                if g <= 0.5 {
                    error!("gyro failed self-test with st_otp != 0");
                    failed = true;
                };
                g
            } else {
                let g = gyro_st_resp[i].abs();
                if g < OTP0_DPS_OFFSET_THRESH {
                    error!("gyro failed self-test with st_otp == 0");
                    failed = true;
                };
                g as f32
            };
            gyro_st_res[i] = g;

            let a = if accel_st_otp[i] != 0.0 {
                let a = accel_st_resp[i] as f32 / accel_st_otp[i];
                if a <= 0.5 || a >= 1.5 {
                    error!("accel failed self-test with st_otp != 0");
                    failed = true;
                };
                a
            } else {
                let a = accel_st_resp[i].abs();
                if a < OTP0_MGEE_OFFSET_MIN_THRESH || a > OTP0_MGEE_OFFSET_MAX_THRESH {
                    error!("accel failed self-test with st_otp == 0");
                    failed = true;
                }
                a as f32
            };
            accel_st_res[i] = a;

            // TODO maybe a map for the diff FS values instead but doubt I'll need it
            if gyro_avg[i].abs() > FS0_DPS_OFFSET_THRESH {
                error!("gyro offsets failed self-test");
                failed = true;
            }
        }

        Ok(SelfTestResult {
            gyro: Vec3::new(gyro_st_res[0], gyro_st_res[1], gyro_st_res[2]),
            accel: Vec3::new(accel_st_res[0], accel_st_res[1], accel_st_res[2]),
            offset: Vec3::new(gyro_avg[0], gyro_avg[1], gyro_avg[2]),
            passed: !failed,
        })
    }
}
