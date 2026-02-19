use bytemuck::{Pod, Zeroable};

pub const ENDPOINT_CONTROL_OUT: u8 = 0x02;
pub const ENDPOINT_CONTROL_IN: u8 = 0x82;
#[allow(dead_code)]
pub const ENDPOINT_STREAM_IN: u8 = 0x81;
pub const ENDPOINT_INTERRUPT_IN: u8 = 0x83;

pub const T265_VID: u16 = 0x8087;
pub const T265_PID: u16 = 0x0B37;

pub const T265_BOOT_VID: u16 = 0x03E7;
pub const T265_BOOT_PID: u16 = 0x2150;

pub const USB_TIMEOUT: std::time::Duration = std::time::Duration::from_millis(10000);

pub const DEV_GET_TIME: u16 = 0x0002;
#[allow(dead_code)]
pub const DEV_GET_SUPPORTED_RAW_STREAMS: u16 = 0x0004;
#[allow(dead_code)]
pub const DEV_RAW_STREAMS_CONTROL: u16 = 0x0005;
pub const DEV_START: u16 = 0x0012;
pub const DEV_STOP: u16 = 0x0013;
#[allow(dead_code)]
pub const DEV_STATUS: u16 = 0x0014;
pub const DEV_GET_POSE: u16 = 0x0015;
#[allow(dead_code)]
pub const DEV_SAMPLE: u16 = 0x0011;
pub const DEV_GET_TEMPERATURE: u16 = 0x0018;
pub const DEV_SET_TEMPERATURE_THRESHOLD: u16 = 0x0019;
pub const SLAM_SET_6DOF_INTERRUPT_RATE: u16 = 0x1005;
pub const SLAM_6DOF_CONTROL: u16 = 0x1006;
#[allow(dead_code)]
pub const DEV_ERROR: u16 = 0x8000;
#[allow(dead_code)]
pub const SLAM_ERROR: u16 = 0x9000;
#[allow(dead_code)]
pub const SLAM_RELOCALIZATION_EVENT: u16 = 0x100E;

// T265 Status Codes
pub const SUCCESS: u16 = 0x0000;
pub const UNSUPPORTED: u16 = 0x0001;
pub const INVALID_REQUEST_LEN: u16 = 0x0002;
pub const INVALID_PARAMETER: u16 = 0x0003;
pub const BUSY: u16 = 0x0004;
pub const HW_NOT_READY: u16 = 0x0005;
pub const NO_DATA: u16 = 0x0006;
pub const DEVICE_FAILURE: u16 = 0x0007;
pub const DEVICE_BUSY: u16 = 0x0008;
pub const TIMEOUT: u16 = 0x000B;
pub const DEVICE_STOPPED: u16 = 0x000C;
pub const TEMPERATURE_WARNING: u16 = 0x0010;
#[allow(dead_code)]
pub const TEMPERATURE_STOP: u16 = 0x0011;

/// Convert a T265 status code to a human-readable error message
pub fn status_to_string(status: u16) -> &'static str {
    match status {
        SUCCESS => "Success",
        UNSUPPORTED => "Unsupported command or feature",
        INVALID_REQUEST_LEN => "Invalid request length",
        INVALID_PARAMETER => "Invalid parameter",
        BUSY => "Device busy",
        HW_NOT_READY => "Hardware not ready",
        NO_DATA => "No data available",
        DEVICE_FAILURE => "Device failure",
        DEVICE_BUSY => "Device busy",
        TIMEOUT => "Operation timed out",
        DEVICE_STOPPED => "Device stopped",
        TEMPERATURE_WARNING => "Temperature warning",
        _ => "Unknown error",
    }
}

pub const SIXDOF_MODE_NORMAL: u8 = 0x00;
pub const SIXDOF_MODE_ENABLE_MAPPING: u8 = 0x02;
pub const SIXDOF_MODE_ENABLE_RELOCALIZATION: u8 = 0x04;
pub const SIXDOF_MODE_DISABLE_JUMPING: u8 = 0x08;

pub const INTERRUPT_RATE_NONE: u8 = 0x0;
pub const INTERRUPT_RATE_FISHEYE: u8 = 0x1;
pub const INTERRUPT_RATE_IMU: u8 = 0x2;

#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct BulkMessageRequestHeader {
    pub dw_length: u32,
    pub w_message_id: u16,
}

#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct BulkMessageResponseHeader {
    pub dw_length: u32,
    pub w_message_id: u16,
    pub w_status: u16,
}

#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct InterruptMessageHeader {
    pub dw_length: u32,
    pub w_message_id: u16,
}

#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct PoseData {
    pub fl_x: f32,
    pub fl_y: f32,
    pub fl_z: f32,
    pub fl_qi: f32,
    pub fl_qj: f32,
    pub fl_qk: f32,
    pub fl_qr: f32,
    pub fl_vx: f32,
    pub fl_vy: f32,
    pub fl_vz: f32,
    pub fl_vax: f32,
    pub fl_vay: f32,
    pub fl_vaz: f32,
    pub fl_ax: f32,
    pub fl_ay: f32,
    pub fl_az: f32,
    pub fl_aax: f32,
    pub fl_aay: f32,
    pub fl_aaz: f32,
    pub ll_nanoseconds: u64,
    pub dw_tracker_confidence: u32,
    pub dw_mapper_confidence: u32,
    pub dw_tracker_state: u32,
}

#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct InterruptMessageGetPose {
    pub header: InterruptMessageHeader,
    pub b_index: u8,
    pub w_reserved: u8,
    pub pose: PoseData,
}

#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct InterruptMessageStatus {
    pub header: InterruptMessageHeader,
    pub w_status: u16,
}

#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct InterruptMessageError {
    pub header: InterruptMessageHeader,
    pub w_status: u16,
}

#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct InterruptMessageSlamError {
    pub header: InterruptMessageHeader,
    pub w_status: u16,
}

#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
#[allow(dead_code)]
pub struct InterruptMessageSlamRelocalizationEvent {
    pub header: InterruptMessageHeader,
    pub ll_nanoseconds: u64,
    pub w_session_id: u16,
}

// Control messages
#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct BulkMessageRequest6DofControl {
    pub header: BulkMessageRequestHeader,
    pub b_enable: u8,
    pub b_mode: u8,
}

#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct BulkMessageResponse6DofControl {
    pub header: BulkMessageResponseHeader,
}

#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct BulkMessageRequestSet6DofInterruptRate {
    pub header: BulkMessageRequestHeader,
    pub b_interrupt_rate: u8,
}

#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct BulkMessageResponseSet6DofInterruptRate {
    pub header: BulkMessageResponseHeader,
}

#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct BulkMessageRequestStart {
    pub header: BulkMessageRequestHeader,
}

#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct BulkMessageResponseStart {
    pub header: BulkMessageResponseHeader,
}

#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct BulkMessageRequestStop {
    pub header: BulkMessageRequestHeader,
}

#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct BulkMessageResponseStop {
    pub header: BulkMessageResponseHeader,
}

#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct BulkMessageRequestGetTime {
    pub header: BulkMessageRequestHeader,
}

#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct BulkMessageResponseGetTime {
    pub header: BulkMessageResponseHeader,
    pub ll_nanoseconds: u64,
}

#[allow(dead_code)]
pub const SENSOR_TYPE_FISHEYE: u8 = 3;
#[allow(dead_code)]
pub const SENSOR_TYPE_GYRO: u8 = 4;
#[allow(dead_code)]
pub const SENSOR_TYPE_ACCELEROMETER: u8 = 5;

#[inline]
#[allow(dead_code)]
pub fn set_sensor_id(sensor_type: u8, sensor_index: u8) -> u8 {
    ((sensor_type & 0x1F) << 0) | ((sensor_index & 0x07) << 5)
}

#[inline]
#[allow(dead_code)]
pub fn get_sensor_type(sensor_id: u8) -> u8 {
    (sensor_id & 0x1F) >> 0
}

#[inline]
#[allow(dead_code)]
pub fn get_sensor_index(sensor_id: u8) -> u8 {
    (sensor_id & 0xE0) >> 5
}

/// Interrupt raw stream header - common header for IMU samples on endpoint 0x83.
/// Follows the 6-byte InterruptMessageHeader when wMessageID == DEV_SAMPLE.
/// Total: 28 bytes.
#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct InterruptMessageRawStreamHeader {
    pub header: InterruptMessageHeader,    // 6 bytes
    pub b_sensor_id: u8,                   // 1 byte: bits 0-4 = type, bits 5-7 = index
    pub b_reserved: u8,                    // 1 byte
    pub ll_nanoseconds: u64,               // 8 bytes: device timestamp
    pub ll_arrival_nanoseconds: u64,       // 8 bytes: arrival timestamp
    pub dw_frame_id: u32,                  // 4 bytes: running counter per sensor
}

/// Metadata payload for an accelerometer or gyroscope interrupt sample.
/// Total: 24 bytes.
#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct InterruptMessageImuStreamMetadata {
    pub dw_metadata_length: u32,   // 4 bytes (always 4)
    pub fl_temperature: f32,        // 4 bytes: degrees Celsius
    pub dw_frame_length: u32,      // 4 bytes (always 12)
    pub fl_x: f32,                  // 4 bytes: accel m/s² or gyro rad/s on X
    pub fl_y: f32,                  // 4 bytes: accel m/s² or gyro rad/s on Y
    pub fl_z: f32,                  // 4 bytes: accel m/s² or gyro rad/s on Z
}

/// Complete interrupt IMU stream message (accelerometer or gyroscope).
/// Total: 52 bytes (28 header + 24 metadata).
#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct InterruptMessageImuStream {
    pub raw_stream_header: InterruptMessageRawStreamHeader, // 28 bytes
    pub metadata: InterruptMessageImuStreamMetadata,        // 24 bytes
}

// Video stream structures
#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
#[allow(dead_code)]
pub struct BulkMessageRawStreamHeader {
    pub header: BulkMessageRequestHeader,
    pub b_sensor_id: u8,
    pub b_reserved: u8,
    pub ll_nanoseconds: u64,
    pub ll_arrival_nanoseconds: u64,
    pub dw_frame_id: u32,
}

#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
#[allow(dead_code)]
pub struct BulkMessageVideoStreamMetadataHeader {
    pub dw_metadata_length: u32,
    pub dw_exposuretime: u32,
    pub f_gain: f32,
    pub dw_frame_length: u32,
}

#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
#[allow(dead_code)]
pub struct SupportedRawStreamMessage {
    pub b_sensor_id: u8,
    pub b_reserved: u8,
    pub w_width: u16,
    pub w_height: u16,
    pub b_pixel_format: u8,
    pub b_output_mode: u8,
    pub w_stride: u16,
    pub w_frames_per_second: u16,
}

#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
#[allow(dead_code)]
pub struct BulkMessageRequestGetSupportedRawStreams {
    pub header: BulkMessageRequestHeader,
}

#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
#[allow(dead_code)]
pub struct BulkMessageResponseGetSupportedRawStreamsHeader {
    pub header: BulkMessageResponseHeader,
    pub w_num_supported_streams: u16,
    pub w_reserved: u16,
}

#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
#[allow(dead_code)]
pub struct BulkMessageRequestRawStreamsControlHeader {
    pub header: BulkMessageRequestHeader,
    pub w_num_enabled_streams: u16,
}

#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
#[allow(dead_code)]
pub struct BulkMessageResponseRawStreamsControl {
    pub header: BulkMessageResponseHeader,
}

// Temperature structs

/// Request body for DEV_GET_TEMPERATURE (0x0018).
/// dwLength = 6, wMessageID = DEV_GET_TEMPERATURE.
#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct BulkMessageRequestGetTemperature {
    pub header: BulkMessageRequestHeader,
}

/// Fixed-size header portion of the DEV_GET_TEMPERATURE response.
/// Followed by `dw_count` × `SensorTemperatureEntry` entries.
#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct BulkMessageResponseGetTemperatureHeader {
    pub header: BulkMessageResponseHeader, // 8 bytes
    pub dw_count: u32,                     // number of entries that follow
}

/// One sensor's temperature data: 12 bytes.
/// dw_index: 0 = VPU, 1 = IMU, 2 = BLE.
#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct SensorTemperatureEntry {
    pub dw_index: u32,
    pub f_temperature: f32, // Celsius
    pub f_threshold: f32,   // threshold Celsius (device stops at this)
}

/// Fixed-size header for DEV_SET_TEMPERATURE_THRESHOLD (0x0019).
/// Followed by `dw_count` × `SensorSetTemperatureEntry` entries.
/// dwLength = (12 + 8 * dwCount).
#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct BulkMessageRequestSetTemperatureThresholdHeader {
    pub header: BulkMessageRequestHeader, // 6 bytes
    pub w_force_token: u16,               // required when threshold is 80–100 °C
    pub dw_count: u32,                    // number of sensor entries that follow
}

/// One sensor's new threshold: 8 bytes.
#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct SensorSetTemperatureEntry {
    pub dw_index: u32,
    pub f_threshold: f32, // new threshold in Celsius
}

/// Response for DEV_SET_TEMPERATURE_THRESHOLD.
#[repr(C, packed)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct BulkMessageResponseSetTemperatureThreshold {
    pub header: BulkMessageResponseHeader,
}
