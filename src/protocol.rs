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
pub const DEV_START: u16 = 0x0012;
pub const DEV_STOP: u16 = 0x0013;
#[allow(dead_code)]
pub const DEV_STATUS: u16 = 0x0014;
pub const DEV_GET_POSE: u16 = 0x0015;
pub const SLAM_SET_6DOF_INTERRUPT_RATE: u16 = 0x1005;
pub const SLAM_6DOF_CONTROL: u16 = 0x1006;
#[allow(dead_code)]
pub const DEV_ERROR: u16 = 0x8000;
#[allow(dead_code)]
pub const SLAM_ERROR: u16 = 0x9000;
#[allow(dead_code)]
pub const SLAM_RELOCALIZATION_EVENT: u16 = 0x100E;

pub const SUCCESS: u16 = 0x0000;
pub const DEVICE_BUSY: u16 = 0x0008;
pub const DEVICE_STOPPED: u16 = 0x000C;
pub const TEMPERATURE_WARNING: u16 = 0x0010;

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
