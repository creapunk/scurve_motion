/// Describes a single motion segment or phase in an S-curve profile.
#[derive(Default, Clone, Copy)]
pub struct MotionPolynomial {
    pub time: f64,
    pub pos: f64,
    pub vel: f64,
    pub acc: f64,
    pub jrk: f64,
}

impl MotionPolynomial {
    /// Creates a new MotionTask.
    pub fn new(time: f64, pos: f64, vel: f64, acc: f64, jrk: f64) -> Self {
        Self {
            time,
            pos,
            vel,
            acc,
            jrk,
        }
    }
}
