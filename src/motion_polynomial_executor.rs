use crate::buffer_fifo::BufferFIFO;
use crate::motion_polynomial::MotionPolynomial;

/// PolyMotion manages a queue of MotionTask items and updates them
/// discretely according to a set frequency.
pub struct MotionPolynomialExecutor<const N: usize> {
    /// Storage for incoming motion tasks
    buffer: BufferFIFO<MotionPolynomial, N>,

    /// The current "active" task
    init: MotionPolynomial,

    /// The current instantaneous state
    inst: MotionPolynomial,

    /// Update frequency in Hz
    freq: f64,

    /// Current time-step in discrete ticks
    time: i32,

    /// Duration of the current task in ticks
    duration: i32,
}

impl<const N: usize> MotionPolynomialExecutor<N> {
    /// Creates a new MotionExecutor with a given frequency (Hz).
    pub fn new(freq: u16) -> Self {
        Self {
            freq: freq as f64,
            time: 1,
            duration: 0,
            buffer: BufferFIFO::default(),
            init: MotionPolynomial::default(),
            inst: MotionPolynomial::default(),
        }
    }

    /// Adds a motion task to the FIFO buffer.
    pub fn add_task(&mut self, task: MotionPolynomial) {
        self.buffer.write(task);
    }

    /// Fetches the next task from the buffer if the current one is done.
    fn get_next(&mut self) {
        if !self.buffer.is_empty() {
            let new_task = self.buffer.read();
            if new_task.time != 0.0 {
                self.init = new_task;
                self.inst = new_task;
                self.inst.time = 0.0;

                self.duration = (new_task.time * self.freq) as i32;
                self.time = 0;
            } else {
                // If the task's time is zero, skip it and read another.
                self.get_next();
            }
        }
    }

    /// Internal math update for the current task.
    fn math(&mut self) {
        let j0 = self.init.jrk;
        let a0 = self.init.acc;
        let v0 = self.init.vel;
        let s0 = self.init.pos;

        let time = self.time as f64 / self.freq;

        // acc(t) = a0 + j0*t
        let acc = a0 + j0 * time;

        // vel(t) = v0 + ∫(0..t) acc(τ)dτ = v0 + a0*t + j0*t^2/2
        let vel = v0 + (a0 + acc) * time * 0.5;

        // pos(t) = s0 + v0*t + a0*t^2/2 + j0*t^3/6
        // Or an equivalent formula using average velocity over time
        let vel_avg = v0 + (2.0 * a0 + acc) * time / 6.0;
        let pos = s0 + vel_avg * time;

        self.inst.acc = acc;
        self.inst.vel = vel;
        self.inst.pos = pos;
        self.inst.time = time;
    }

    /// Increments the time step by one, updating internal state or moving to the next task.
    pub fn tick(&mut self) {
        if self.time > self.duration {
            self.get_next();
        } else {
            self.time += 1;
            self.math();
        }
    }

    /// Sets a new frequency in Hz.
    pub fn set_freq(&mut self, freq: i16) {
        self.freq = freq as f64;
    }

    /// Returns the current acceleration.
    pub fn get_acc(&self) -> f64 {
        self.inst.acc
    }

    /// Returns the current velocity.
    pub fn get_vel(&self) -> f64 {
        self.inst.vel
    }

    /// Returns the current position.
    pub fn get_pos(&self) -> f64 {
        self.inst.pos
    }

    /// Checks if the buffer is full.
    pub fn is_full(&self) -> bool {
        self.buffer.is_full()
    }
}
