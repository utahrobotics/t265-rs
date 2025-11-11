use crate::error::{Error, Result};
use crate::pose::{Confidence, Pose, TrackerState};
use crate::protocol::*;
use crate::video::VideoFrame;
use rusb::{DeviceHandle, GlobalContext};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::sync::Arc;
use std::time::Instant;

pub struct T265Device {
    handle: Arc<DeviceHandle<GlobalContext>>,
    device_id: String,
    time_offset_ns: i64,
    pose_streaming: Arc<AtomicBool>,
    current_mode: u8,
    video_streaming: Arc<AtomicBool>,
}

impl T265Device {
    pub fn new(handle: DeviceHandle<GlobalContext>, device_id: String) -> Self {
        Self {
            handle: Arc::new(handle),
            device_id,
            time_offset_ns: 0,
            pose_streaming: Arc::new(AtomicBool::new(false)),
            current_mode: SIXDOF_MODE_ENABLE_MAPPING | SIXDOF_MODE_ENABLE_RELOCALIZATION,
            video_streaming: Arc::new(AtomicBool::new(false)),
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
                // Return detailed error with human-readable message
                return Err(Error::from_status(status));
            }
        }

        Ok(resp)
    }

    pub(crate) fn enable_6dof(&mut self, mode: u8) -> Result<()> {
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
    #[allow(dead_code)]
    pub(crate) fn set_interrupt_rate(&self, rate: u8) -> Result<()> {
        let req = BulkMessageRequestSet6DofInterruptRate {
            header: BulkMessageRequestHeader {
                dw_length: 5,
                w_message_id: SLAM_SET_6DOF_INTERRUPT_RATE,
            },
            b_interrupt_rate: rate,
        };

        let _resp: BulkMessageResponseSet6DofInterruptRate = self.bulk_request(&req)?;
        Ok(())
    }

    pub(crate) fn start_streaming(&self) -> Result<()> {
        let req = BulkMessageRequestStart {
            header: BulkMessageRequestHeader {
                dw_length: 4,
                w_message_id: DEV_START,
            },
        };

        let _resp: BulkMessageResponseStart = self.bulk_request(&req)?;
        Ok(())
    }

    pub(crate) fn stop_streaming(&self) -> Result<()> {
        let req = BulkMessageRequestStop {
            header: BulkMessageRequestHeader {
                dw_length: 4,
                w_message_id: DEV_STOP,
            },
        };

        let _resp: BulkMessageResponseStop = self.bulk_request(&req)?;
        Ok(())
    }

    pub(crate) fn sync_time(&mut self) -> Result<()> {
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

    pub(crate) fn start_pose_stream(&mut self, tx: mpsc::Sender<Pose>) -> Result<()> {
        self.enable_6dof(self.current_mode)?;
        self.start_streaming()?;

        let handle = self.handle.clone();
        let device_id = self.device_id.clone();
        let time_offset = self.time_offset_ns;
        let pose_streaming = Arc::clone(&self.pose_streaming);
        pose_streaming.store(true, Ordering::SeqCst);

        std::thread::spawn(move || {
            let mut buffer = vec![0u8; 128];

            while pose_streaming.load(Ordering::SeqCst) {
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
                                        if pose_streaming.load(Ordering::SeqCst) {
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

    pub(crate) fn stop_pose_stream(&mut self) -> Result<()> {
        self.pose_streaming.store(false, Ordering::SeqCst);
        std::thread::sleep(std::time::Duration::from_millis(200));
        self.stop_streaming()?;
        Ok(())
    }

    pub(crate) fn get_supported_video_streams(&self) -> Result<Vec<SupportedRawStreamMessage>> {
        let request = BulkMessageRequestGetSupportedRawStreams {
            header: BulkMessageRequestHeader {
                dw_length: std::mem::size_of::<BulkMessageRequestGetSupportedRawStreams>() as u32,
                w_message_id: DEV_GET_SUPPORTED_RAW_STREAMS,
            },
        };

        let req_bytes = bytemuck::bytes_of(&request);
        self.handle
            .write_bulk(ENDPOINT_CONTROL_OUT, req_bytes, USB_TIMEOUT)?;

        let mut buffer = vec![0u8; 1024];
        let bytes_read = self
            .handle
            .read_bulk(ENDPOINT_CONTROL_IN, &mut buffer, USB_TIMEOUT)?;

        if bytes_read < std::mem::size_of::<BulkMessageResponseGetSupportedRawStreamsHeader>() {
            return Err(Error::Protocol(
                "Response too short for GetSupportedRawStreams".to_string(),
            ));
        }

        let header: BulkMessageResponseGetSupportedRawStreamsHeader = bytemuck::pod_read_unaligned(
            &buffer[..std::mem::size_of::<BulkMessageResponseGetSupportedRawStreamsHeader>()],
        );

        if header.header.w_status != SUCCESS {
            return Err(Error::from_status(header.header.w_status));
        }

        let num_streams = header.w_num_supported_streams as usize;
        let streams_offset = std::mem::size_of::<BulkMessageResponseGetSupportedRawStreamsHeader>();
        let stream_size = std::mem::size_of::<SupportedRawStreamMessage>();

        let mut streams = Vec::new();
        for i in 0..num_streams {
            let offset = streams_offset + (i * stream_size);
            if offset + stream_size > bytes_read {
                break;
            }
            let stream: SupportedRawStreamMessage =
                bytemuck::pod_read_unaligned(&buffer[offset..offset + stream_size]);
            streams.push(stream);
        }

        Ok(streams)
    }

    pub(crate) fn enable_video_streams(&self, streams: &[SupportedRawStreamMessage]) -> Result<()> {
        if streams.is_empty() {
            return Ok(());
        }

        let header_size = std::mem::size_of::<BulkMessageRequestRawStreamsControlHeader>();
        let stream_size = std::mem::size_of::<SupportedRawStreamMessage>();
        let total_size = header_size + (streams.len() * stream_size);

        let mut buffer = vec![0u8; total_size];

        let request_header = BulkMessageRequestRawStreamsControlHeader {
            header: BulkMessageRequestHeader {
                dw_length: total_size as u32,
                w_message_id: DEV_RAW_STREAMS_CONTROL,
            },
            w_num_enabled_streams: streams.len() as u16,
        };

        buffer[..header_size].copy_from_slice(bytemuck::bytes_of(&request_header));

        for (i, stream) in streams.iter().enumerate() {
            let offset = header_size + (i * stream_size);
            buffer[offset..offset + stream_size].copy_from_slice(bytemuck::bytes_of(stream));
        }

        self.handle
            .write_bulk(ENDPOINT_CONTROL_OUT, &buffer, USB_TIMEOUT)?;

        let mut resp_buffer = vec![0u8; 64];
        let bytes_read =
            self.handle
                .read_bulk(ENDPOINT_CONTROL_IN, &mut resp_buffer, USB_TIMEOUT)?;

        if bytes_read < std::mem::size_of::<BulkMessageResponseRawStreamsControl>() {
            return Err(Error::Protocol(
                "Response too short for RawStreamsControl".to_string(),
            ));
        }

        let response: BulkMessageResponseRawStreamsControl = bytemuck::pod_read_unaligned(
            &resp_buffer[..std::mem::size_of::<BulkMessageResponseRawStreamsControl>()],
        );

        if response.header.w_status != SUCCESS {
            return Err(Error::from_status(response.header.w_status));
        }

        Ok(())
    }

    pub(crate) fn start_video_stream(&self) -> Result<mpsc::Receiver<VideoFrame>> {
        if self.video_streaming.load(Ordering::SeqCst) {
            return Err(Error::DeviceAlreadyOpen);
        }

        // Start device streaming if pose stream hasn't already started it
        if !self.pose_streaming.load(Ordering::SeqCst) {
            self.start_streaming()?;
        }

        let (tx, rx) = mpsc::channel();
        let handle = self.handle.clone();
        let device_id = self.device_id.clone();
        let video_streaming = self.video_streaming.clone();
        let time_offset_ns = self.time_offset_ns;

        video_streaming.store(true, Ordering::SeqCst);

        std::thread::spawn(move || {
            const MAX_FRAME_SIZE: usize = 848 * 800 + 1024;
            let mut buffer = vec![0u8; MAX_FRAME_SIZE];

            while video_streaming.load(Ordering::SeqCst) {
                match handle.read_bulk(
                    ENDPOINT_STREAM_IN,
                    &mut buffer,
                    std::time::Duration::from_millis(1000),
                ) {
                    Ok(size) if size >= std::mem::size_of::<BulkMessageRequestHeader>() => {
                        let header: BulkMessageRequestHeader = bytemuck::pod_read_unaligned(
                            &buffer[..std::mem::size_of::<BulkMessageRequestHeader>()],
                        );

                        let msg_id = header.w_message_id;

                        match msg_id {
                            DEV_SAMPLE
                                if size >= std::mem::size_of::<BulkMessageRawStreamHeader>() =>
                            {
                                let stream_header: BulkMessageRawStreamHeader =
                                    bytemuck::pod_read_unaligned(
                                        &buffer
                                            [..std::mem::size_of::<BulkMessageRawStreamHeader>()],
                                    );

                                let sensor_type = get_sensor_type(stream_header.b_sensor_id);
                                let sensor_index = get_sensor_index(stream_header.b_sensor_id);

                                if sensor_type == SENSOR_TYPE_FISHEYE {
                                    let metadata_offset =
                                        std::mem::size_of::<BulkMessageRawStreamHeader>();

                                    if size
                                        >= metadata_offset
                                            + std::mem::size_of::<
                                                BulkMessageVideoStreamMetadataHeader,
                                            >()
                                    {
                                        let metadata: BulkMessageVideoStreamMetadataHeader =
                                            bytemuck::pod_read_unaligned(
                                                &buffer[metadata_offset
                                                    ..metadata_offset
                                                        + std::mem::size_of::<
                                                            BulkMessageVideoStreamMetadataHeader,
                                                        >(
                                                        )],
                                            );

                                        let data_offset = metadata_offset
                                            + std::mem::size_of::<
                                                BulkMessageVideoStreamMetadataHeader,
                                            >();
                                        let frame_length = metadata.dw_frame_length as usize;

                                        if data_offset + frame_length <= size {
                                            let frame_data = buffer
                                                [data_offset..data_offset + frame_length]
                                                .to_vec();

                                            let width = 848u16;
                                            let height = 800u16;
                                            let stride = width;

                                            let video_frame = VideoFrame {
                                                sensor_index,
                                                timestamp_ns: (stream_header.ll_nanoseconds as i64
                                                    + time_offset_ns)
                                                    as u64,
                                                arrival_timestamp_ns: (stream_header
                                                    .ll_arrival_nanoseconds
                                                    as i64
                                                    + time_offset_ns)
                                                    as u64,
                                                frame_id: stream_header.dw_frame_id,
                                                width,
                                                height,
                                                stride,
                                                exposure_time_us: metadata.dw_exposuretime,
                                                gain: metadata.f_gain,
                                                data: frame_data,
                                            };

                                            if tx.send(video_frame).is_err() {
                                                eprintln!(
                                                    "Device {} video receiver dropped, stopping",
                                                    device_id
                                                );
                                                break;
                                            }
                                        }
                                    }
                                }
                            }
                            DEV_STATUS => {
                                if size >= 8 {
                                    let status_msg: InterruptMessageStatus =
                                        bytemuck::pod_read_unaligned(&buffer[..8]);
                                    if status_msg.w_status == DEVICE_STOPPED {
                                        if video_streaming.load(Ordering::SeqCst) {
                                            eprintln!(
                                                "Device {} received stale video STOPPED message, ignoring",
                                                device_id
                                            );
                                        } else {
                                            eprintln!("Device {} video stream stopped", device_id);
                                            break;
                                        }
                                    }
                                }
                            }
                            _ => {
                                eprintln!("Unknown message: {msg_id}");
                            }
                        }
                    }
                    Ok(_) => {
                        // short read, continue
                    }
                    Err(rusb::Error::Timeout) => {
                        eprintln!("timeout");
                    }
                    Err(e) => {
                        eprintln!("Device {} video stream error: {}", device_id, e);
                        break;
                    }
                }
            }

            video_streaming.store(false, Ordering::SeqCst);
        });

        Ok(rx)
    }

    pub(crate) fn stop_video_stream(&self) -> Result<()> {
        if !self.video_streaming.load(Ordering::SeqCst) {
            return Ok(());
        }
        self.video_streaming.store(false, Ordering::SeqCst);
        std::thread::sleep(std::time::Duration::from_millis(200));

        // If pose stream is not running, stop device streaming
        // (video stream started it, so video stream should stop it)
        if !self.pose_streaming.load(Ordering::SeqCst) {
            self.stop_streaming()?;
        }

        Ok(())
    }
}

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
