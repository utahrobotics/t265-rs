/// Which temperature sensor on the T265.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TempSensor {
    /// Vision Processing Unit (VPU)
    Vpu = 0,
    /// Inertial Measurement Unit (IMU)
    Imu = 1,
    /// Bluetooth Low Energy chip (BLE)
    Ble = 2,
}

impl TempSensor {
    pub fn from_index(idx: u32) -> Option<Self> {
        match idx {
            0 => Some(TempSensor::Vpu),
            1 => Some(TempSensor::Imu),
            2 => Some(TempSensor::Ble),
            _ => None,
        }
    }
}

impl std::fmt::Display for TempSensor {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TempSensor::Vpu => write!(f, "VPU"),
            TempSensor::Imu => write!(f, "IMU"),
            TempSensor::Ble => write!(f, "BLE"),
        }
    }
}

/// Temperature reading from a single T265 sensor.
#[derive(Debug, Clone)]
pub struct SensorTemperature {
    /// Which sensor this reading is from.
    pub sensor: TempSensor,
    /// Current temperature in degrees Celsius.
    pub temperature_c: f32,
    /// Shutdown threshold in degrees Celsius.
    /// The device sends TEMPERATURE_WARNING at 90% of this value
    /// and stops all streaming when the threshold is reached.
    pub threshold_c: f32,
}
