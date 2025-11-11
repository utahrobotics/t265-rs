/// Video frame data from T265 fisheye cameras
#[derive(Clone, Debug)]
pub struct VideoFrame {
    ///(0 = left fisheye, 1 = right fisheye)
    pub sensor_index: u8,
    /// Frame timestamp in nanoseconds since device initialization
    pub timestamp_ns: u64,
    /// Frame arrival timestamp in nanoseconds since device initialization
    pub arrival_timestamp_ns: u64,
    /// Frame ID (running counter per camera)
    pub frame_id: u32,
    /// Image width in pixels
    pub width: u16,
    /// Image height in pixels
    pub height: u16,
    /// Image stride in bytes (bytes per row, may include padding)
    pub stride: u16,
    /// Exposure time in microseconds
    pub exposure_time_us: u32,
    /// Gain multiplier
    pub gain: f32,
    pub data: Vec<u8>,
}

impl VideoFrame {
    /// Get a specific pixel value (0-255) at coordinates (x, y)
    /// Returns None if coordinates are out of bounds
    pub fn get_pixel(&self, x: u16, y: u16) -> Option<u8> {
        if x >= self.width || y >= self.height {
            return None;
        }
        let offset = (y as usize) * (self.stride as usize) + (x as usize);
        self.data.get(offset).copied()
    }

    pub fn get_row(&self, y: u16) -> Option<&[u8]> {
        if y >= self.height {
            return None;
        }
        let offset = (y as usize) * (self.stride as usize);
        let end = offset + (self.width as usize);
        self.data.get(offset..end)
    }
}
