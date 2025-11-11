use crate::error::{Error, Result};
use rusb::{DeviceHandle, GlobalContext};
use sha1::{Digest, Sha1};
use std::time::Duration;

const FIRMWARE_VERSION: &str = "0.2.0.951";
const FIRMWARE_SHA1: &str = "c3940ccbb0e3045603e4aceaa2d73427f96e24bc";
// is this a copyright violation? we may never know.
const FIRMWARE_DATA: &[u8] = include_bytes!("../firmware/target-0.2.0.951.mvcmd");

const BOOTLOADER_ENDPOINT_OUT: u8 = 0x01;

fn verify_sha1(data: &[u8]) -> bool {
    let mut hasher = Sha1::new();
    hasher.update(data);
    let result = hasher.finalize();
    let hash_str = format!("{:x}", result);
    hash_str == FIRMWARE_SHA1
}

pub fn get_firmware() -> Result<&'static [u8]> {
    println!("Using embedded T265 firmware v{}", FIRMWARE_VERSION);
    println!("Size: {} MB", FIRMWARE_DATA.len() / 1_000_000);

    if !verify_sha1(FIRMWARE_DATA) {
        return Err(Error::Io(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            "Embedded firmware SHA1 verification failed!",
        )));
    }
    Ok(FIRMWARE_DATA)
}

/// Upload firmware to T265 device in bootloader mode
/// Device must be in boot loader mode or you might brick it?
pub fn upload_firmware(handle: &DeviceHandle<GlobalContext>) -> Result<()> {
    let firmware = get_firmware()?;

    println!("\nUploading firmware to device...");

    const CHUNK_SIZE: usize = 64 * 1024;
    let mut _transferred = 0;

    for chunk in firmware.chunks(CHUNK_SIZE) {
        match handle.write_bulk(BOOTLOADER_ENDPOINT_OUT, chunk, Duration::from_secs(5)) {
            Ok(n) => {
                _transferred += n;
            }
            Err(e) => {
                return Err(Error::Usb(e));
            }
        }
    }
    std::thread::sleep(Duration::from_secs(3));

    Ok(())
}
