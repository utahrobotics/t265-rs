#[derive(Debug, Clone, PartialEq)]
pub struct Pose {
    pub translation: [f32; 3],
    pub rotation: [f32; 4],
    pub velocity: [f32; 3],
    pub angular_velocity: [f32; 3],
    pub acceleration: [f32; 3],
    pub angular_acceleration: [f32; 3],
    pub timestamp_ns: u64,
    pub tracker_confidence: Confidence,
    // Mapper confidence always says failed even if it seems to be working in practice
    pub mapper_confidence: Confidence,
    pub tracker_state: TrackerState,
    pub device_id: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Confidence {
    Failed = 0,
    Low = 1,
    Medium = 2,
    High = 3,
}

impl From<u32> for Confidence {
    fn from(value: u32) -> Self {
        match value & 0x3 {
            0 => Confidence::Failed,
            1 => Confidence::Low,
            2 => Confidence::Medium,
            3 => Confidence::High,
            _ => Confidence::Failed,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrackerState {
    Inactive = 0x0,
    Active3DOF = 0x3,
    Active6DOF = 0x4,
    InertialOnly3DOF = 0x7,
    Unknown,
}

impl From<u32> for TrackerState {
    fn from(value: u32) -> Self {
        match value {
            0x0 => TrackerState::Inactive,
            0x3 => TrackerState::Active3DOF,
            0x4 => TrackerState::Active6DOF,
            0x7 => TrackerState::InertialOnly3DOF,
            _ => TrackerState::Unknown,
        }
    }
}
