use t265_rs::T265Manager;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut manager = T265Manager::new()?;
    let device_ids = manager.discover_devices()?;
    
    let device_id = &device_ids[0];
    let pose_rx = manager.start_pose_stream(device_id)?;

    loop {
        let pose = pose_rx.recv()?;
        println!(
            "pos: [{:.3}, {:.3}, {:.3}] rot: [{:.3}, {:.3}, {:.3}, {:.3}]",
            pose.translation[0],
            pose.translation[1],
            pose.translation[2],
            pose.rotation[0],
            pose.rotation[1],
            pose.rotation[2],
            pose.rotation[3]
        );
    }
}
