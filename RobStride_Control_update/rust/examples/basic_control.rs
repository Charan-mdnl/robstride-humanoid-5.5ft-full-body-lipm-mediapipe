/*
 * RobStride Rust Basic Control Example
 * Demonstrates basic motor control operations
 */

use std::env;
use std::f64::consts::PI;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

// Import from main module
use socketcan::{CanSocket, CanFrame, EmbeddedFrame, CanId};

// Include the control functions from main.rs
include!("../main.rs");

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let motor_id: u8 = env::args()
        .nth(1)
        .unwrap_or("11".to_string())
        .parse()
        .expect("Invalid motor ID");

    println!("🎯 RobStride Rust Basic Control Example");
    println!("Using Motor ID: {}", motor_id);

    // Create CAN socket
    let socket = Arc::new(Mutex::new(
        CanSocket::open("can0")?
    ));

    println!("✅ CAN interface initialized");

    // Initialize motor
    println!("⚡ Enabling motor...");
    enable_motor(&socket, motor_id)?;
    thread::sleep(Duration::from_millis(500));

    println!("⚙️ Setting MIT mode...");
    set_mode_raw(&socket, motor_id, 0)?;
    thread::sleep(Duration::from_millis(200));

    println!("📊 Setting limits...");
    write_limit(&socket, motor_id, PARAM_VELOCITY_LIMIT, 20.0)?;
    write_limit(&socket, motor_id, PARAM_TORQUE_LIMIT, 20.0)?;

    // Test movements
    println!("🎮 Starting test movements...");

    let movements = vec![
        (0.0, "Home"),
        (PI/2.0, "90°"),
        (-PI/2.0, "-90°"),
        (0.0, "Home"),
    ];

    for (position, description) in movements {
        println!("📍 Moving to {}", description);

        let socket = socket.lock().unwrap();
        write_operation_frame(&socket, motor_id, position, 30.0, 0.5)?;
        drop(socket);

        thread::sleep(Duration::from_secs(2));
    }

    println!("✅ Test completed successfully!");
    Ok(())
}