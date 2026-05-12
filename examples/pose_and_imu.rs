use std::time::Duration;
use t265_rs::{ImuKind, Result, T265Manager};

fn main() -> Result<()> {
    let mut manager = T265Manager::new()?;
    let devices = manager.discover_devices()?;

    if devices.is_empty() {
        eprintln!("No T265 devices found");
        return Ok(());
    }

    println!("Found {} T265 device(s):", devices.len());
    for id in &devices {
        println!("  {}", id);
    }
    println!();

    // Enable IMU streams first - must happen before DEV_START (start_all_pose_streams)
    let imu_rx = manager.start_all_imu_streams()?;
    let pose_rx = manager.start_all_pose_streams()?;

    println!("Streaming pose + IMU. Press Ctrl-C to stop.\n");

    let deadline = std::time::Instant::now() + Duration::from_secs(100);

    while std::time::Instant::now() < deadline {
        // Drain all pending IMU frames
        while let Ok(imu) = imu_rx.try_recv() {
            match imu.kind {
                ImuKind::Accel => println!(
                    "[{}] ACCEL  {:8.4} {:8.4} {:8.4} m/s²   temp {:.1}°C  frame {}",
                    imu.sample.device_id,
                    imu.sample.x,
                    imu.sample.y,
                    imu.sample.z,
                    imu.sample.temperature,
                    imu.sample.frame_id,
                ),
                ImuKind::Gyro => println!(
                    "[{}] GYRO   {:8.4} {:8.4} {:8.4} rad/s  temp {:.1}°C  frame {}",
                    imu.sample.device_id,
                    imu.sample.x,
                    imu.sample.y,
                    imu.sample.z,
                    imu.sample.temperature,
                    imu.sample.frame_id,
                ),
            }
        }

        // Print the latest pose if one is ready
        if let Ok(Ok(pose)) = pose_rx.recv_timeout(Duration::from_millis(5)) {
            println!(
                "[{}] POSE   pos=[{:6.3} {:6.3} {:6.3}] m  \
                 vel=[{:6.3} {:6.3} {:6.3}] m/s  conf={:?}",
                pose.device_id,
                pose.translation[0],
                pose.translation[1],
                pose.translation[2],
                pose.velocity[0],
                pose.velocity[1],
                pose.velocity[2],
                pose.tracker_confidence,
            );
        }
    }

    Ok(())
}
