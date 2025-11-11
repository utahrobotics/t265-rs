use std::fs;
use std::io::Write;
use t265_rs::T265Manager;

fn save_frame_as_pgm(frame: &t265_rs::VideoFrame, filename: &str) -> std::io::Result<()> {
    let mut file = fs::File::create(filename)?;
    writeln!(file, "P5\n{} {}\n255", frame.width, frame.height)?;
    for y in 0..frame.height {
        if let Some(row) = frame.get_row(y) {
            file.write_all(row)?;
        }
    }
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut manager = T265Manager::new()?;
    manager.discover_devices()?;

    let devices = manager.devices();
    if devices.is_empty() {
        eprintln!("no T265 devices found.");
        return Ok(());
    }

    // Enable all video streams (both fisheye cameras)
    manager.enable_all_video_streams()?;

    let video_rx = manager.start_all_video_streams()?;
    fs::create_dir_all("frames")?;

    let mut saved = 0;
    while saved < 10 {
        if let Ok((device_id, frame)) = video_rx.recv_timeout(std::time::Duration::from_millis(500))
        {
            let filename = format!(
                "frames/device{}_cam{}_frame{:06}.pgm",
                device_id, frame.sensor_index, frame.frame_id
            );
            if let Err(e) = save_frame_as_pgm(&frame, &filename) {
                eprintln!("Error saving frame: {}", e);
            } else {
                println!("Saved {}", filename);
                saved += 1;
            }
        }
    }

    manager.stop_all_video_streams()?;

    println!("Saved {} frames.", saved);
    Ok(())
}
