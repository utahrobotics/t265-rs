use thiserror::Error;

#[derive(Debug, Error)]
pub enum Error {
    #[error("USB error: {0}")]
    Usb(#[from] rusb::Error),

    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),

    #[error("Device not found")]
    DeviceNotFound,

    #[error("Invalid message format")]
    InvalidMessage,

    #[error("Unexpected message ID: {0}")]
    UnexpectedMessage(u16),

    #[error("Device busy")]
    DeviceBusy,

    #[error("Device stopped")]
    DeviceStopped,

    #[error("Temperature warning")]
    TemperatureWarning,

    #[error("Device error: status code {0:#x}")]
    DeviceError(u16),

    #[error("Device status: {0:#x}")]
    DeviceStatus(u16),

    #[error("Command failed with status: {0}")]
    CommandFailed(u16),

    #[error("Interface not claimed")]
    InterfaceNotClaimed,

    #[error("Message too short: expected {expected}, got {actual}")]
    MessageTooShort { expected: usize, actual: usize },
}

pub type Result<T> = std::result::Result<T, Error>;
