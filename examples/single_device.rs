use t265_rs::{Result, T265Manager};

fn main() -> Result<()> {
    let mut manager = T265Manager::new()?;
    let devices = manager.discover_devices()?;

    if devices.is_empty() {
        println!("No T265 devices found!");
        return Ok(());
    }

    println!("Found {} T265 device(s):", devices.len());
    for (i, id) in devices.iter().enumerate() {
        println!("  {}. {}", i + 1, id);
    }
    println!();

    let device_id = &devices[0];

    // Start device for synchronous pose reading
    manager.start_device_for_sync_read(device_id, t265_rs::SIXDOF_MODE_NORMAL)?;

    println!("\nReading 10 poses:\n");

    // Get device reference for read_pose()
    let device = manager.get_device_by_id(device_id).unwrap();

    for i in 0..10 {
        match device.read_pose() {
            Ok(pose) => {
                println!("Pose {}:", i + 1);
                println!(
                    "  Position: [{:.3}, {:.3}, {:.3}] m",
                    pose.translation[0], pose.translation[1], pose.translation[2]
                );
                println!(
                    "  Rotation: [{:.3}, {:.3}, {:.3}, {:.3}]",
                    pose.rotation[0], pose.rotation[1], pose.rotation[2], pose.rotation[3]
                );
                println!(
                    "  Velocity: [{:.3}, {:.3}, {:.3}] m/s",
                    pose.velocity[0], pose.velocity[1], pose.velocity[2]
                );
                println!("  Confidence: {:?}", pose.tracker_confidence);
                println!("  State: {:?}", pose.tracker_state);
                println!();
            }
            Err(e) => {
                eprintln!("Error reading pose: {}", e);
            }
        }
    }

    println!("Stopping streaming...");
    manager.stop_device_sync_read(device_id)?;

    println!("Done!");

    Ok(())
}
