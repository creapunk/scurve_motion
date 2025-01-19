use std::error::Error;
use scurve_motion;
// use gnuplot::{Figure, Caption, Color};
use gnuplot::*;
fn main() -> Result<(), Box<dyn Error>> {
    // -----------------------
    // 1. Set up parameters
    // -----------------------
    // For demonstration, we pick simple values:
    let pos_0 = 0.0;   // Initial position
    let pos_1 = 50.0;  // Target position
    let vel_0 = 0.0;   // Initial velocity
    let vel_1 = 0.0;   // Target velocity

    // Jerks for 3 segments of motion (start, middle, end).
    // If your library expects them to be positive or can handle sign flipping
    // internally, set them accordingly. We'll just keep them identical here.
    let jrk_0 = 20.0;
    let jrk_1 = 20.0;
    let jrk_2 = 20.0;

    // Motion limits
    let v_lim = 20.0; // Maximum velocity
    let a_lim = 10.0; // Maximum acceleration
    let j_lim = 40.0; // Maximum jerk

    // -------------------------
    // 2. Create and configure
    // -------------------------
    let mut my_scurve = scurve_motion::SCurve::default();

    // Set motion limits, return early if something goes wrong
    if !my_scurve.set_limits(v_lim, a_lim, j_lim) {
        return Err("Failed to set motion limits. Check your values.".into());
    }

    // Set the "goal": positions, velocities, jerks
    if !my_scurve.set_goal((pos_0, pos_1), (vel_0, vel_1), (jrk_0, jrk_1, jrk_2)) {
        return Err("Failed to set motion goal. Possibly invalid jerk/velocity constraints.".into());
    }

    // ---------------------
    // 3. Process S-curve
    // ---------------------
    // You can call process(true) for a "const-velocity-allowed" trajectory
    // or process(false) for "smooth" transitions that skip constant velocity.
    let constraints_result = my_scurve.process(true);
    let total_time = constraints_result.duration as f64;

    // Basic sanity check
    if total_time <= 0.0 {
        return Err("Calculated total motion time is non-positive. Check inputs.".into());
    }

    // -------------------------
    // 4. Set up data sampling
    // -------------------------
    let sampling_rate = 1000.0; // points per second (for example)
    let num_points = (sampling_rate * total_time).ceil() as usize;

    // Create an executor to generate discrete trajectory data
    let mut executor = scurve_motion::MotionPolynomialExecutor::<10>::new(sampling_rate as u16);
    let phases = constraints_result.s_curve(1e-6); // 1e-6 is an example threshold

    // Add the "phases" of motion
    for phase in phases {
        executor.add_task(phase);
    }

    // Prepare containers for results
    let mut time_axis = Vec::with_capacity(num_points);
    let mut positions = Vec::with_capacity(num_points);
    let mut velocities = Vec::with_capacity(num_points);
    let mut accelerations = Vec::with_capacity(num_points);

    // --------------------------------
    // 5. Fill trajectory data at each step
    // --------------------------------
    for i in 0..num_points {
        let t = i as f64 / sampling_rate;
        time_axis.push(t);

        // Move executor one tick
        if t <= total_time {
            executor.tick();
        }

        // Obtain current pos, vel, acc
        positions.push(executor.get_pos() as f64);
        velocities.push(executor.get_vel() as f64);
        accelerations.push(executor.get_acc() as f64);
    }

    // Quick final check (did we roughly reach target position?)
    // Tolerances can be refined based on your application
    let final_position = *positions.last().unwrap_or(&0.0);
    let position_error = (final_position - pos_1).abs();
    if position_error > 0.01 {
        eprintln!("Warning: final position is off by more than 0.01 units.");
        // Not returning an error, just alerting. Adjust logic as needed.
    }

    // --------------
    // 6. Plot data
    // --------------
    // We'll create a single figure with multiple sub-plots:
    //  * Position vs. time
    //  * Velocity vs. time
    //  * Acceleration vs. time
    // This uses the "gnuplot" crate, which must be in [dependencies].
    // For production code, you might want to save to a file with `.set_terminal("pngcairo", ...)`.

    let mut fg = Figure::new();

    // Set up a multiplot layout with 3 rows and 1 column

    // ----- Position Plot -----
    {
        let axes = fg.axes2d();
        axes.set_title("Position, Velocity, Acceleration vs. Time", &[]);
        axes.set_x_label("Time (s)", &[]);
        axes.set_y_label("Position derivatives", &[]);
        axes.lines(&time_axis, &positions, &[Color("blue"), Caption("Position")]);
        axes.lines(&time_axis, &velocities, &[Color("red"), Caption("Velocity")]);
        axes.lines(&time_axis, &accelerations, &[Color("green"), Caption("Acceleration")]);
    }

    // Show the figure in a window (if your environment supports it)
    // or save it to a file. For example:
    // fg.set_terminal("pngcairo", "scurve_result.png");
    // fg.show(); // or fg.savefig("scurve_result.png");

    // Attempt to show in a pop-up window (might require gnuplot installed)
    fg.show().map_err(|e| format!("Failed to display plot: {e}"))?;

    println!("Plot generated. Total motion time: {:.3} seconds.", total_time);
    Ok(())
}