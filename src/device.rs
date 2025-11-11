use crate::error::{Error, Result};
use crate::pose::{Confidence, Pose, TrackerState};
use crate::protocol::*;
use rusb::{DeviceHandle, GlobalContext};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::sync::Arc;
use std::time::Instant;

pub struct T265Device {
    handle: Arc<DeviceHandle<GlobalContext>>,
    device_id: String,
    time_offset_ns: i64,
    running: Arc<AtomicBool>,
    current_mode: u8,
}

impl T265Device {
    pub fn new(handle: DeviceHandle<GlobalContext>, device_id: String) -> Self {
        Self {
            handle: Arc::new(handle),
            device_id,
            time_offset_ns: 0,
            running: Arc::new(AtomicBool::new(false)),
            current_mode: SIXDOF_MODE_ENABLE_MAPPING | SIXDOF_MODE_ENABLE_RELOCALIZATION,
        }
    }

    pub fn device_id(&self) -> &str {
        &self.device_id
    }

    /// Set the 6DOF mode that will be used when starting the pose stream.
    /// This must be called before start_pose_stream().
    pub fn set_mode(&mut self, mode: u8) {
        self.current_mode = mode;
    }

    /// Get the current 6DOF mode
    pub fn get_mode(&self) -> u8 {
        self.current_mode
    }

    fn bulk_request<Req: bytemuck::Pod, Resp: bytemuck::Pod>(&self, request: &Req) -> Result<Resp> {
        let req_bytes = bytemuck::bytes_of(request);

        self.handle
            .write_bulk(ENDPOINT_CONTROL_OUT, req_bytes, USB_TIMEOUT)?;

        // bigger buffer than we probably need
        let mut resp_buf = vec![0u8; 1024];
        let size = self
            .handle
            .read_bulk(ENDPOINT_CONTROL_IN, &mut resp_buf, USB_TIMEOUT)?;

        if size < std::mem::size_of::<Resp>() {
            return Err(Error::MessageTooShort {
                expected: std::mem::size_of::<Resp>(),
                actual: size,
            });
        }

        let resp: Resp = bytemuck::pod_read_unaligned(&resp_buf[..std::mem::size_of::<Resp>()]);

        if std::mem::size_of::<Resp>() >= std::mem::size_of::<BulkMessageResponseHeader>() {
            let header: BulkMessageResponseHeader = bytemuck::pod_read_unaligned(
                &resp_buf[..std::mem::size_of::<BulkMessageResponseHeader>()],
            );
            let status = header.w_status;
            if status != SUCCESS {
                if status == DEVICE_BUSY {
                    return Err(Error::DeviceBusy);
                }
                // some other error idk
                return Err(Error::CommandFailed(status));
            }
        }

        Ok(resp)
    }

    pub fn enable_6dof(&mut self, mode: u8) -> Result<()> {
        let req = BulkMessageRequest6DofControl {
            header: BulkMessageRequestHeader {
                dw_length: 9,
                w_message_id: SLAM_6DOF_CONTROL,
            },
            b_enable: 1,
            b_mode: mode,
        };

        let _resp: BulkMessageResponse6DofControl = self.bulk_request(&req)?;
        self.current_mode = mode; // Store the current mode
        Ok(())
    }

    /// I dont think this works
    pub fn set_interrupt_rate(&self, rate: u8) -> Result<()> {
        let req = BulkMessageRequestSet6DofInterruptRate {
            header: BulkMessageRequestHeader {
                dw_length: 7,
                w_message_id: SLAM_SET_6DOF_INTERRUPT_RATE,
            },
            b_interrupt_rate: rate,
        };

        let _resp: BulkMessageResponseSet6DofInterruptRate = self.bulk_request(&req)?;
        Ok(())
    }

    pub fn start_streaming(&self) -> Result<()> {
        let req = BulkMessageRequestStart {
            header: BulkMessageRequestHeader {
                dw_length: 6,
                w_message_id: DEV_START,
            },
        };

        let _resp: BulkMessageResponseStart = self.bulk_request(&req)?;
        Ok(())
    }

    pub fn stop_streaming(&self) -> Result<()> {
        let req = BulkMessageRequestStop {
            header: BulkMessageRequestHeader {
                dw_length: 6,
                w_message_id: DEV_STOP,
            },
        };

        let _resp: BulkMessageResponseStop = self.bulk_request(&req)?;
        Ok(())
    }

    pub fn sync_time(&mut self) -> Result<()> {
        let host_start = Instant::now();

        let req = BulkMessageRequestGetTime {
            header: BulkMessageRequestHeader {
                dw_length: 6,
                w_message_id: DEV_GET_TIME,
            },
        };

        let resp: BulkMessageResponseGetTime = self.bulk_request(&req)?;
        let device_time = resp.ll_nanoseconds;

        let host_end = Instant::now();
        let roundtrip = host_end - host_start;

        let host_mid_ns = (host_start.elapsed() + roundtrip / 2).as_nanos() as i64;
        let device_ns = device_time as i64;

        self.time_offset_ns = host_mid_ns - device_ns;
        Ok(())
    }

    pub fn read_pose(&self) -> Result<Pose> {
        let mut buffer = vec![0u8; 128];

        loop {
            let size =
                self.handle
                    .read_interrupt(ENDPOINT_INTERRUPT_IN, &mut buffer, USB_TIMEOUT)?;

            if size < std::mem::size_of::<InterruptMessageHeader>() {
                return Err(Error::MessageTooShort {
                    expected: std::mem::size_of::<InterruptMessageHeader>(),
                    actual: size,
                });
            }

            let header: InterruptMessageHeader = bytemuck::pod_read_unaligned(
                &buffer[..std::mem::size_of::<InterruptMessageHeader>()],
            );

            match header.w_message_id {
                DEV_GET_POSE => {
                    let expected_size = std::mem::size_of::<InterruptMessageGetPose>();
                    if size < expected_size {
                        return Err(Error::MessageTooShort {
                            expected: expected_size,
                            actual: size,
                        });
                    }

                    let msg: InterruptMessageGetPose =
                        bytemuck::pod_read_unaligned(&buffer[..expected_size]);
                    return Ok(self.convert_pose_data(&msg.pose));
                }
                DEV_ERROR => {
                    if size >= 8 {
                        let msg: InterruptMessageError = bytemuck::pod_read_unaligned(&buffer[..8]);
                        return Err(Error::DeviceError(msg.w_status));
                    }
                    return Err(Error::DeviceError(0));
                }
                SLAM_ERROR => {
                    if size >= 8 {
                        let msg: InterruptMessageSlamError =
                            bytemuck::pod_read_unaligned(&buffer[..8]);
                        let status = msg.w_status;
                        eprintln!("Warning: SLAM error {:#x}, continuing...", status);
                        continue;
                    }
                    continue;
                }
                SLAM_RELOCALIZATION_EVENT => {
                    if size >= 18 {
                        let msg: InterruptMessageSlamRelocalizationEvent =
                            bytemuck::pod_read_unaligned(&buffer[..18]);
                        let timestamp_ns = msg.ll_nanoseconds;
                        let session_id = msg.w_session_id;
                        if session_id == 0 {
                            eprintln!(
                                "Relocalization: Recovered position within current session at {} ns",
                                timestamp_ns
                            );
                        } else {
                            eprintln!(
                                "Relocalization: Recovered position from previous session {} at {} ns",
                                session_id, timestamp_ns
                            );
                        }
                        continue;
                    }
                    continue;
                }
                DEV_STATUS => {
                    if size >= 8 {
                        let msg: InterruptMessageStatus =
                            bytemuck::pod_read_unaligned(&buffer[..8]);
                        let status = msg.w_status;
                        match status {
                            DEVICE_STOPPED => return Err(Error::DeviceStopped),
                            TEMPERATURE_WARNING => return Err(Error::TemperatureWarning),
                            _ => {
                                eprintln!(
                                    "Warning: Unknown device status {:#x}, continuing...",
                                    status
                                );
                                continue;
                            }
                        }
                    }
                    continue;
                }
                _ => {
                    let msg_id = header.w_message_id;
                    eprintln!(
                        "Warning: Unknown interrupt message ID {:#x}, skipping...",
                        msg_id
                    );
                    continue;
                }
            }
        }
    }

    fn convert_pose_data(&self, data: &PoseData) -> Pose {
        Pose {
            translation: [data.fl_x, data.fl_y, data.fl_z],
            rotation: [data.fl_qi, data.fl_qj, data.fl_qk, data.fl_qr],
            velocity: [data.fl_vx, data.fl_vy, data.fl_vz],
            angular_velocity: [data.fl_vax, data.fl_vay, data.fl_vaz],
            acceleration: [data.fl_ax, data.fl_ay, data.fl_az],
            angular_acceleration: [data.fl_aax, data.fl_aay, data.fl_aaz],
            timestamp_ns: (data.ll_nanoseconds as i64 + self.time_offset_ns) as u64,
            tracker_confidence: Confidence::from(data.dw_tracker_confidence),
            mapper_confidence: Confidence::from(data.dw_mapper_confidence), // Use raw value like librealsense
            tracker_state: TrackerState::from(data.dw_tracker_state),
            device_id: self.device_id.clone(),
        }
    }

    pub fn start_pose_stream(&mut self, tx: mpsc::Sender<Pose>) -> Result<()> {
        // Use the current mode instead of always resetting to NORMAL
        self.enable_6dof(self.current_mode)?;
        self.start_streaming()?;

        let handle = self.handle.clone();
        let device_id = self.device_id.clone();
        let time_offset = self.time_offset_ns;
        let running = Arc::clone(&self.running);
        running.store(true, Ordering::SeqCst);

        std::thread::spawn(move || {
            let mut buffer = vec![0u8; 128];

            while running.load(Ordering::SeqCst) {
                match handle.read_interrupt(
                    ENDPOINT_INTERRUPT_IN,
                    &mut buffer,
                    std::time::Duration::from_millis(1000),
                ) {
                    Ok(size) if size >= std::mem::size_of::<InterruptMessageHeader>() => {
                        let header: InterruptMessageHeader = bytemuck::pod_read_unaligned(
                            &buffer[..std::mem::size_of::<InterruptMessageHeader>()],
                        );
                        let msg_id = header.w_message_id;

                        match msg_id {
                            DEV_GET_POSE
                                if size >= std::mem::size_of::<InterruptMessageGetPose>() =>
                            {
                                let expected_size = std::mem::size_of::<InterruptMessageGetPose>();
                                let msg: InterruptMessageGetPose =
                                    bytemuck::pod_read_unaligned(&buffer[..expected_size]);
                                let pose =
                                    convert_pose_data_static(&msg.pose, time_offset, &device_id);
                                if tx.send(pose).is_err() {
                                    break;
                                }
                            }
                            DEV_ERROR if size >= 8 => {
                                let msg: InterruptMessageError =
                                    bytemuck::pod_read_unaligned(&buffer[..8]);
                                let status = msg.w_status;
                                eprintln!("Device {} error: {:#x}", device_id, status);
                            }
                            SLAM_ERROR if size >= 8 => {
                                let msg: InterruptMessageSlamError =
                                    bytemuck::pod_read_unaligned(&buffer[..8]);
                                let status = msg.w_status;
                                eprintln!("Device {} SLAM error: {:#x}", device_id, status);
                            }
                            SLAM_RELOCALIZATION_EVENT if size >= 18 => {
                                let msg: InterruptMessageSlamRelocalizationEvent =
                                    bytemuck::pod_read_unaligned(&buffer[..18]);
                                let timestamp_ns = msg.ll_nanoseconds;
                                let session_id = msg.w_session_id;
                                if session_id == 0 {
                                    eprintln!(
                                        "Device {} Relocalization: Recovered position within current session at {} ns",
                                        device_id, timestamp_ns
                                    );
                                } else {
                                    eprintln!(
                                        "Device {} Relocalization: Recovered position from previous session {} at {} ns",
                                        device_id, session_id, timestamp_ns
                                    );
                                }
                            }
                            DEV_STATUS if size >= 8 => {
                                let msg: InterruptMessageStatus =
                                    bytemuck::pod_read_unaligned(&buffer[..8]);
                                let status = msg.w_status;
                                match status {
                                    DEVICE_STOPPED => {
                                        // Check if we're actually supposed to be running
                                        // If running is true, this is a stale message from a previous stop
                                        if running.load(Ordering::SeqCst) {
                                            eprintln!("Device {} received stale STOPPED message, ignoring", device_id);
                                        } else {
                                            eprintln!("Device {} stopped", device_id);
                                            break;
                                        }
                                    }
                                    TEMPERATURE_WARNING => {
                                        eprintln!("Device {} temperature warning", device_id);
                                    }
                                    _ => {
                                        eprintln!("Device {device_id} reported unknown status message: {status}");
                                    }
                                }
                            }
                            _ => {
                                eprintln!("Device {device_id} reported unknown or incomplete msg id: {msg_id}");
                            }
                        }
                    }
                    Err(rusb::Error::Timeout) => continue,
                    Err(e) => {
                        eprintln!("Read error on device {}: {:?}", device_id, e);
                        break;
                    }
                    _ => {}
                }
            }
        });

        Ok(())
    }

    pub fn stop_pose_stream(&mut self) -> Result<()> {
        self.running.store(false, Ordering::SeqCst);
        std::thread::sleep(std::time::Duration::from_millis(200));
        self.stop_streaming()?;
        Ok(())
    }
}

/// wont take ownership of self
fn convert_pose_data_static(data: &PoseData, time_offset_ns: i64, device_id: &str) -> Pose {
    Pose {
        translation: [data.fl_x, data.fl_y, data.fl_z],
        rotation: [data.fl_qi, data.fl_qj, data.fl_qk, data.fl_qr],
        velocity: [data.fl_vx, data.fl_vy, data.fl_vz],
        angular_velocity: [data.fl_vax, data.fl_vay, data.fl_vaz],
        acceleration: [data.fl_ax, data.fl_ay, data.fl_az],
        angular_acceleration: [data.fl_aax, data.fl_aay, data.fl_aaz],
        timestamp_ns: (data.ll_nanoseconds as i64 + time_offset_ns) as u64,
        tracker_confidence: Confidence::from(data.dw_tracker_confidence),
        mapper_confidence: Confidence::from(data.dw_mapper_confidence),
        tracker_state: TrackerState::from(data.dw_tracker_state),
        device_id: device_id.to_string(),
    }
}

impl Drop for T265Device {
    fn drop(&mut self) {
        let _ = self.stop_pose_stream();
    }
}
