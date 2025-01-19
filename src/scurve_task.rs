use crate::motion_polynomial::MotionPolynomial;

/// The SCurveTaskExecutor holds final computed phases and time segments
/// after the main S-curve computation is done.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SCurveTask {
    pub pos: (f64, f64),
    pub vel: (f64, f64),
    pub acc: (f64, f64),
    pub jrk: (f64, f64, f64, f64),
    pub vel_lim: f64,
    pub duration: f64,
}

impl Default for SCurveTask {
    fn default() -> Self {
        Self {
            pos: (0.0, 0.0),
            vel: (0.0, 0.0),
            acc: (0.0, 0.0),
            jrk: (0.0, 0.0, 0.0, 0.0),
            vel_lim: 0.0,
            duration: 0.0,
        }
    }
}

impl SCurveTask {
    /// Creates a new SCurveTaskExecutor with given parameters.
    pub fn new(
        pos: (f64, f64),
        vel: (f64, f64),
        acc: (f64, f64),
        jrk: (f64, f64, f64, f64),
        vel_lim: f64,
        duration: f64,
    ) -> Self {
        Self {
            pos,
            vel,
            acc,
            jrk,
            vel_lim,
            duration,
        }
    }

    /// Example helper function that demonstrates how partial distances / velocities can be calculated.
    fn calc_jerk_s_v(vel: f64, jrk: f64, t: f64) -> (f64, f64) {
        // start acc = 0
        // v = v0 + j*t^2/2
        let dv = jrk * t * t * 0.5;
        let v = vel + dv;
        // s = v0*t + j*t^3/6 = t*(v0 + dv/3)
        let s = (vel + dv / 3.0) * t;
        (s, v)
    }

    /// Compute half of an S-curve with jerk up and jerk down phases.
    pub fn calc_half_s(
        v_init: f64,
        v_exit: f64,
        j_init: f64,
        j_exit: f64,
        a_max: f64,
        threshold: f64,
    ) -> (MotionPolynomial, MotionPolynomial, MotionPolynomial, f64) {
        // PHASE 1 - JERKED INCR ACCEL
        let mut t01 = 0.0;
        let mut s01 = 0.0;
        let mut v1 = v_init;
        let phase1_duration = a_max / j_init;
        if phase1_duration > threshold {
            t01 = phase1_duration;
            (s01, v1) = Self::calc_jerk_s_v(v_init, j_init, t01);
        }

        // PHASE 3 - JERKED DECR ACCEL
        let mut v2 = v_exit;
        let mut s23 = 0.0;
        let mut t12 = 0.0;
        let mut s12 = 0.0;
        let mut t23 = 0.0;

        let phase3_duration = a_max / j_exit;
        if phase3_duration > threshold {
            t23 = phase3_duration;
            (s23, v2) = Self::calc_jerk_s_v(v_exit, -j_exit, t23);
        }

        // PHASE 2 - CONST ACCEL
        let phase2_duration = (v2 - v1) / a_max;
        if phase2_duration > threshold {
            t12 = phase2_duration;
            s12 = (v1 + a_max * t12 * 0.5) * t12;
        }

        let path = s01 + s12 + s23;
        let mt1 = MotionPolynomial::new(t01, s01, v_init, 0.0, j_init);
        let mt2 = MotionPolynomial::new(t12, s12, v1, a_max, 0.0);
        let mt3 = MotionPolynomial::new(t23, s23, v2, a_max, -j_exit);

        (mt1, mt2, mt3, path)
    }

    /// Full 7-phase S-curve breakdown as an example (not fully integrated with SCurve above).
    pub fn s_curve(&self, dt: f64) -> [MotionPolynomial; 7] {
        let threshold = dt * 0.5;
        let mut mt: [MotionPolynomial; 7] = [MotionPolynomial::default(); 7];

        let path = self.pos.1 - self.pos.0;
        let mut s03 = 0.0;
        if self.vel.0 != self.vel_lim {
            (mt[0], mt[1], mt[2], s03) = Self::calc_half_s(
                self.vel.0,
                self.vel_lim,
                self.jrk.0,
                self.jrk.1,
                self.acc.0,
                threshold,
            );
        }

        let mut s34 = path - s03;
        if self.vel.1 != self.vel_lim {
            let s47;
            (mt[4], mt[5], mt[6], s47) = Self::calc_half_s(
                self.vel_lim,
                self.vel.1,
                -self.jrk.2,
                -self.jrk.3,
                -self.acc.1,
                threshold,
            );
            s34 -= s47;
        }

        let t34 = s34 / self.vel_lim;
        if t34 > threshold {
            mt[3] = MotionPolynomial::new(t34, s34, self.vel_lim, 0.0, 0.0);
        }

        let mut pos = self.pos.0;
        for motion_task in &mut mt {
            let task_ds = motion_task.pos;
            motion_task.pos = pos;
            pos += task_ds;
        }
        // println!(
        //     "T1: {:.8}, T2: {:.8}, T3: {:.8}, T4: {:.8}, T5: {:.8}, T6: {:.8}, T7: {:.8}",
        //     mt[0].time, mt[1].time, mt[2].time, mt[3].time, mt[4].time, mt[5].time, mt[6].time
        // );
        mt
    }
}
