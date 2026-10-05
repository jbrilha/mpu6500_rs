#[derive(Debug, Clone, Copy, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Vec3<T> {
    pub x: T,
    pub y: T,
    pub z: T,
}

impl<T> Vec3<T> {
    pub const fn new(x: T, y: T, z: T) -> Self {
        Self { x, y, z }
    }

    pub fn map<U>(self, f: impl Fn(T) -> U) -> Vec3<U> {
        Vec3 {
            x: f(self.x),
            y: f(self.y),
            z: f(self.z),
        }
    }

    pub fn zip<U, V>(self, other: Vec3<U>, f: impl Fn(T, U) -> V) -> Vec3<V> {
        Vec3 {
            x: f(self.x, other.x),
            y: f(self.y, other.y),
            z: f(self.z, other.z),
        }
    }
}

impl Vec3<i16> {
    pub const fn from_be_bytes(raw: [u8; 6]) -> Self {
        Self {
            x: i16::from_be_bytes([raw[0], raw[1]]),
            y: i16::from_be_bytes([raw[2], raw[3]]),
            z: i16::from_be_bytes([raw[4], raw[5]]),
        }
    }

    pub fn widen(self) -> Vec3<i32> {
        self.map(|v| v as i32)
    }
}

impl Vec3<i32> {
    pub fn sub(self, other: Vec3<i32>) -> Vec3<i32> {
        self.zip(other, |a, b| a - b)
    }

    pub fn scale(self, lsb_per_unit: f32) -> Vec3<f32> {
        self.map(|v| v as f32 / lsb_per_unit)
    }
}

/// burst of the contiguous output block of accel, temp and gyro
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Sample {
    pub accel: Vec3<i16>,
    pub temp: i16,
    pub gyro: Vec3<i16>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum GyroFs {
    Dps250 = 0,
    Dps500 = 1,
    Dps1000 = 2,
    Dps2000 = 3,
}

impl GyroFs {
    pub const fn lsb_per_dps(self) -> f32 {
        match self {
            GyroFs::Dps250 => 131.0,
            GyroFs::Dps500 => 65.5,
            GyroFs::Dps1000 => 32.8,
            GyroFs::Dps2000 => 16.4,
        }
    }

    pub const fn bits(self) -> u8 {
        (self as u8) << 3
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum AccelFs {
    G2 = 0,
    G4 = 1,
    G8 = 2,
    G16 = 3,
}

impl AccelFs {
    pub const fn lsb_per_g(self) -> f32 {
        match self {
            AccelFs::G2 => 16384.0,
            AccelFs::G4 => 8192.0,
            AccelFs::G8 => 4096.0,
            AccelFs::G16 => 2048.0,
        }
    }

    pub const fn bits(self) -> u8 {
        (self as u8) << 3
    }
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SelfTestResult {
    pub gyro: Vec3<f32>,
    pub accel: Vec3<f32>,
    pub offset: Vec3<i32>,
    pub passed: bool,
}
