use crate::device::T265Device;
use crate::error::{Error, Result};
use crate::firmware;
use crate::pose::Pose;
use crate::protocol::{
    SupportedRawStreamMessage, T265_BOOT_PID, T265_BOOT_VID, T265_PID, T265_VID,
};
use crate::video::VideoFrame;
use rusb::{GlobalContext, UsbContext};
use std::sync::mpsc;

pub struct T265Manager {
    devices: Vec<T265Device>,
    context: GlobalContext,
}

impl T265Manager {
    pub fn new() -> Result<Self> {
        Ok(Self {
            devices: Vec::new(),
            context: GlobalContext::default(),
        })
    }

    pub fn discover_devices(&mut self) -> Result<Vec<String>> {
        // we may want to disable firmware loading at some point hence having two functions
        self.discover_devices_with_options(true)
    }

    pub fn discover_devices_with_options(&mut self, auto_boot: bool) -> Result<Vec<String>> {
        let mut device_ids = Vec::new();
        let mut bootloader_device_infos = Vec::new();

        let device_list = self.context.devices()?;

        for device in device_list.iter() {
            let device_desc = device.device_descriptor()?;

            if device_desc.vendor_id() == T265_BOOT_VID && device_desc.product_id() == T265_BOOT_PID
            {
                bootloader_device_infos.push((device.bus_number(), device.address()));
            }

            if device_desc.vendor_id() == T265_VID && device_desc.product_id() == T265_PID {
                let handle = device.open()?;

                let timeout = std::time::Duration::from_secs(1);
                let languages = handle.read_languages(timeout)?;
                let serial = if let Some(lang) = languages.first() {
                    handle
                        .read_serial_number_string(*lang, &device_desc, timeout)
                        .unwrap_or_else(|_| format!("unknown_{}", device_ids.len()))
                } else {
                    format!("unknown_{}", device_ids.len())
                };

                handle.claim_interface(0)?;

                let device_id = serial.clone();
                let t265_device = T265Device::new(handle, device_id.clone());

                self.devices.push(t265_device);
                device_ids.push(device_id);
            }
        }

        // Handle devices stuck in boot loader mode
        if !bootloader_device_infos.is_empty() {
            if auto_boot {
                for (bus, addr) in bootloader_device_infos {
                    let device_list = self.context.devices()?;
                    for device in device_list.iter() {
                        if device.bus_number() == bus && device.address() == addr {
                            let handle = device.open()?;
                            handle.claim_interface(0)?;
                            println!("\nBooting device on bus {} address {}...", bus, addr);
                            firmware::upload_firmware(&handle)?;
                            break;
                        }
                    }
                }

                println!("Re Enumerating after boot");
                self.devices.clear();
                std::thread::sleep(std::time::Duration::from_secs(2));
                return self.discover_devices_with_options(false);
            } else {
                eprintln!(
                    "\nSkipping {} device(s) stuck in bootloader.",
                    bootloader_device_infos.len()
                );
                eprintln!("You may use rs-enumerate-devices to boot them manually.\n");
            }
        }

        Ok(device_ids)
    }

    pub fn open_device(&mut self, device_id: &str) -> Result<()> {
        for device in self.context.devices()?.iter() {
            let device_desc = device.device_descriptor()?;

            if device_desc.vendor_id() == T265_VID && device_desc.product_id() == T265_PID {
                let handle = device.open()?;

                let timeout = std::time::Duration::from_secs(1);
                let languages = handle.read_languages(timeout)?;
                let serial = if let Some(lang) = languages.first() {
                    handle
                        .read_serial_number_string(*lang, &device_desc, timeout)
                        .unwrap_or_else(|_| format!("unknown_{}", self.devices.len()))
                } else {
                    format!("unknown_{}", self.devices.len())
                };

                if serial == device_id {
                    handle.claim_interface(0)?;
                    let t265_device = T265Device::new(handle, device_id.to_string());
                    self.devices.push(t265_device);
                    return Ok(());
                }
            }
        }

        Err(Error::DeviceNotFound)
    }

    pub fn get_device(&self, device_id: &str) -> Option<&T265Device> {
        self.devices.iter().find(|d| d.device_id() == device_id)
    }

    pub fn get_device_mut(&mut self, device_id: &str) -> Option<&mut T265Device> {
        self.devices.iter_mut().find(|d| d.device_id() == device_id)
    }

    pub fn devices(&self) -> &[T265Device] {
        &self.devices
    }

    pub fn devices_mut(&mut self) -> &mut [T265Device] {
        &mut self.devices
    }

    /// Start Pose stream for all devices
    pub fn start_all_pose_streams(&mut self) -> Result<mpsc::Receiver<Pose>> {
        let (tx, rx) = mpsc::channel();

        for device in &mut self.devices {
            device.sync_time()?;
        }

        for device in &mut self.devices {
            device.start_pose_stream(tx.clone())?;
        }

        Ok(rx)
    }

    pub fn stop_all_pose_streams(&mut self) -> Result<()> {
        for device in &mut self.devices {
            device.stop_pose_stream()?;
        }
        Ok(())
    }

    /// Returns a map of device_id -> supported streams
    pub fn get_supported_video_streams(
        &self,
    ) -> Result<std::collections::HashMap<String, Vec<SupportedRawStreamMessage>>> {
        let mut result = std::collections::HashMap::new();
        for device in &self.devices {
            let streams = device.get_supported_video_streams()?;
            result.insert(device.device_id().to_string(), streams);
        }
        Ok(result)
    }

    /// Enable video streams on a specific device
    pub fn enable_video_streams(
        &self,
        device_id: &str,
        // the supported raw stream message type can be found from the get_supported_video_streams() function
        streams: &[SupportedRawStreamMessage],
    ) -> Result<()> {
        let device = self.get_device(device_id).ok_or(Error::DeviceNotFound)?;
        device.enable_video_streams(streams)
    }

    /// Enable all video streams on all devices
    pub fn enable_all_video_streams(&self) -> Result<()> {
        for device in &self.devices {
            let streams = device.get_supported_video_streams()?;

            let mut fisheye_streams: Vec<_> = streams
                .into_iter()
                .filter(|stream| {
                    let sensor_type = crate::protocol::get_sensor_type(stream.b_sensor_id);
                    sensor_type == crate::protocol::SENSOR_TYPE_FISHEYE
                })
                .collect();

            for stream in &mut fisheye_streams {
                stream.b_output_mode = 1;
            }

            device.enable_video_streams(&fisheye_streams)?;
        }
        Ok(())
    }

    pub fn start_video_stream(&self, device_id: &str) -> Result<mpsc::Receiver<VideoFrame>> {
        let device = self.get_device(device_id).ok_or(Error::DeviceNotFound)?;
        device.start_video_stream()
    }

    pub fn start_all_video_streams(&self) -> Result<mpsc::Receiver<(String, VideoFrame)>> {
        let (tx, rx) = mpsc::channel();

        for device in &self.devices {
            let device_rx = device.start_video_stream()?;
            let device_id = device.device_id().to_string();
            let tx_clone = tx.clone();

            std::thread::spawn(move || {
                while let Ok(frame) = device_rx.recv() {
                    if tx_clone.send((device_id.clone(), frame)).is_err() {
                        break;
                    }
                }
            });
        }

        Ok(rx)
    }

    /// Stop video streaming on a specific device
    pub fn stop_video_stream(&self, device_id: &str) -> Result<()> {
        let device = self.get_device(device_id).ok_or(Error::DeviceNotFound)?;
        device.stop_video_stream()
    }

    /// Stop video streaming on all devices
    pub fn stop_all_video_streams(&self) -> Result<()> {
        for device in &self.devices {
            device.stop_video_stream()?;
        }
        Ok(())
    }

    /// Set the 6DOF mode for a specific device. Must be called before starting pose stream.
    /// possible modes:
    /// ```rust
    /// pub const SIXDOF_MODE_NORMAL: u8 = 0x00;
    /// pub const SIXDOF_MODE_ENABLE_MAPPING: u8 = 0x02;
    /// pub const SIXDOF_MODE_ENABLE_RELOCALIZATION: u8 = 0x04;
    /// pub const SIXDOF_MODE_DISABLE_JUMPING: u8 = 0x08;
    /// ```
    pub fn set_device_mode(&mut self, device_id: &str, mode: u8) -> Result<()> {
        let device = self
            .get_device_mut(device_id)
            .ok_or(Error::DeviceNotFound)?;
        device.set_mode(mode);
        Ok(())
    }

    /// Set the 6DOF mode for all devices. Must be called before starting pose stream.
    pub fn set_all_device_modes(&mut self, mode: u8) -> Result<()> {
        for device in &mut self.devices {
            device.set_mode(mode);
        }
        Ok(())
    }

    pub fn get_device_by_id(&self, device_id: &str) -> Option<&T265Device> {
        self.get_device(device_id)
    }

    pub fn get_device_by_id_mut(&mut self, device_id: &str) -> Option<&mut T265Device> {
        self.get_device_mut(device_id)
    }

    pub fn start_device_for_sync_read(&mut self, device_id: &str, mode: u8) -> Result<()> {
        let device = self
            .get_device_mut(device_id)
            .ok_or(Error::DeviceNotFound)?;
        device.sync_time()?;
        device.enable_6dof(mode)?;
        device.start_streaming()?;
        Ok(())
    }

    pub fn stop_device_sync_read(&mut self, device_id: &str) -> Result<()> {
        let device = self
            .get_device_mut(device_id)
            .ok_or(Error::DeviceNotFound)?;
        device.stop_streaming()?;
        Ok(())
    }
}

impl Drop for T265Manager {
    fn drop(&mut self) {
        let _ = self.stop_all_pose_streams();
        let _ = self.stop_all_video_streams();
    }
}
