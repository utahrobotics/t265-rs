use t265_rs::{Result, T265Manager};

fn main() -> Result<()> {
    let mut manager = T265Manager::new()?;
    let devices = manager.discover_devices()?;

    if devices.is_empty() {
        eprintln!("No T265 devices found");
        return Ok(());
    }

    for device_id in &devices {
        let temps = manager.get_temperature(device_id)?;
        println!("Device {}:", device_id);
        for t in &temps {
            println!(
                "  {:3}  {:5.1}°C  (threshold {:.1}°C)",
                t.sensor, t.temperature_c, t.threshold_c
            );
        }
    }

    Ok(())
}
