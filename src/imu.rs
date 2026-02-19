/// Whether this IMU sample is from the accelerometer or gyroscope.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImuKind {
    /// Linear acceleration in m/s²
    Accel,
    /// Angular velocity in rad/s
    Gyro,
}

#[derive(Debug, Clone)]
pub struct ImuSample {
    /// Zero-based index of this sensor within its type (T265 has one of each)
    pub sensor_index: u8,
    /// Timestamp in host nanoseconds (after time-sync offset applied)
    pub timestamp_ns: u64,
    /// Running frame counter for this sensor, starts at 0
    pub frame_id: u32,
    /// Sensor temperature in degrees Celsius
    pub temperature: f32,
    /// X axis: acceleration (m/s²) for Accel, angular velocity (rad/s) for Gyro
    pub x: f32,
    /// Y axis: acceleration (m/s²) for Accel, angular velocity (rad/s) for Gyro
    pub y: f32,
    /// Z axis: acceleration (m/s²) for Accel, angular velocity (rad/s) for Gyro
    pub z: f32,
    pub device_id: String,
}

#[derive(Debug, Clone)]
pub struct ImuFrame {
    pub kind: ImuKind,
    pub sample: ImuSample,
}
