use t265_rs::{Result, T265Manager};

fn main() -> Result<()> {
    let mut manager = T265Manager::new()?;
    manager.discover_devices()?;

    if manager.devices().is_empty() {
        eprintln!("No T265 devices found");
        return Ok(());
    }

    manager.enable_all_video_streams()?;

    let pose_rx = manager.start_all_pose_streams()?;
    let video_rx = manager.start_all_video_streams()?;

    for i in 0..100 {
        if let Ok(pose) = pose_rx.recv_timeout(std::time::Duration::from_millis(50)) {
            println!(
                "Pose {}: pos=[{:.2}, {:.2}, {:.2}] conf={:?}",
                i,
                pose.translation[0],
                pose.translation[1],
                pose.translation[2],
                pose.tracker_confidence
            );
        }

        while let Ok((device_id, frame)) = video_rx.try_recv() {
            println!(
                "[{}] Frame: {}x{} sensor={} frame_id={}",
                device_id, frame.width, frame.height, frame.sensor_index, frame.frame_id
            );
        }
    }

    Ok(())
}
