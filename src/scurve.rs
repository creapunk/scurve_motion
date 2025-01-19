use crate::scurve_task::SCurveTask;

/// The SCurve struct holds the motion constraints and state values for
/// computing an S-curve profile between two positions/velocities.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SCurve {
    /// (pos_init, pos_exit)
    /// The distance to travel is pos_exit - pos_init and must be reached.
    pos: (f64, f64),

    /// (vel_init, vel_exit)
    /// The motion must start and end at these velocity values.
    vel: (f64, f64),

    /// (j_init, j_mid, j_exit)
    ///
    /// j_init: Jerk during initial velocity phase,
    ///
    /// j_mid: Jerk used to smooth the max velocity phase (if reached),
    ///
    /// j_exit: Jerk during final velocity change.
    jrk: (f64, f64, f64),

    /// Position accuracy in binary search and other algos. Less the value -> slower the algo
    epsilon: f64,

    /// Absolute velocity limit that must never be exceeded.
    vel_lim: f64,

    /// Absolute acceleration limit that must never be exceeded.
    acc_lim: f64,

    /// Absolute jerk limit that must never be exceeded.
    jrk_lim: f64,

    /// Task executor object containing computed phases (external struct).
    task: SCurveTask,

    is_lim_valid: bool,
    is_goal_valid: bool,
}

/// Provide a default SCurve with zeroed fields.
impl Default for SCurve {
    fn default() -> Self {
        SCurve {
            pos: (0.0, 0.0),             // Default start/end positions
            vel: (0.0, 0.0),             // Default start/end velocities
            jrk: (0.0, 0.0, 0.0),        // Default jerk values
            vel_lim: 0.0,                // Default velocity limit
            acc_lim: 0.0,                // Default acceleration limit
            jrk_lim: 0.0,                // Default jerk limit
            task: SCurveTask::default(), // Default external task struct
            epsilon: 1e-20,
            is_lim_valid: false,
            is_goal_valid: false,
        }
    }
}

impl SCurve {
    // Depth of binary search used for numeric approximation
    const BINARY_SEARCH_DEPTH: usize = 100;

    // Physical constraints to avoid math overflow
    const T_MIN: f64 = 1e-12;
    const T_MAX: f64 = 31556736.0; // 1 year in seconds
    const S_MIN: f64 = 1e-15;
    const P_MAX: f64 = 1e30;
    const V_MAX: f64 = 1e10;
    const A_MAX: f64 = 1e20;
    const J_MAX: f64 = 1e40;

    /// Creates a new SCurve object with provided parameters,
    /// while enforcing safe numeric clamping and feasibility checks.
    /// Epsilon is used as precession for binary search if required
    pub fn new(
        vel_limit: f64, // max velocity limit
        acc_limit: f64, // max acceleration limit
        jrk_limit: f64, // max jerk limit
        epsilon: f64,   // accuracy of position search
    ) -> Self {
        let mut scurve = SCurve::default();
        scurve.set_limits(vel_limit, acc_limit, jrk_limit);
        scurve.epsilon = if epsilon.is_finite() && epsilon > Self::S_MIN {
            epsilon
        } else {
            Self::S_MIN
        };
        scurve
    }

    pub fn is_valid(&self) -> bool {
        self.is_goal_valid && self.is_lim_valid
    }

    /// Main method to compute the S-curve. If `const_vel` is true,
    /// the code enforces a constant velocity phase (if possible).
    ///
    /// # Detailed explanation of the logic:
    ///
    /// 1. **Check for Default SCurve**. If already in the default state, return (task, 0).
    ///
    /// 2. **Preparation of Initial Data**:
    ///    - Store the initial and exit positions in the task.
    ///    - Store the initial and exit velocities in the task.
    ///    - Calculate `direction`: the direction of movement (signum).
    ///    - Convert everything to the positive axis (working with magnitudes to make algorithms universal).
    ///
    /// 3. **7-Phase Profile (Full S-curve)**:
    ///    - First, attempt to apply 7 phases (acceleration, smoothing, acceleration to v_lim, smoothing, constant velocity, smoothing, deceleration).
    ///    - If during the calculation process (via `calc_six_phase_t_s_a`) it turns out that reaching `v_lim` is feasible, then 7 phases can work.
    ///      - If `const_vel = true`, then add an explicit constant velocity phase.
    ///      - Otherwise, adjust `j_mid` to exactly fit the required distance (via `fit_j_mid`).
    ///
    /// 4. **If 7 Phases are Impossible** (meaning we cannot reach the specified `v_lim` within the available distance),
    ///    - Check if 3 phases are sufficient for changing the velocity (initial → exit).
    ///    - If sufficient (method `calc_dv_shift_t_s_a`), then it’s simply “accelerate (or decelerate) and that’s it”.
    ///
    /// 5. **If 3 Phases Do Not Cover the Entire Distance** (more distance is needed), then proceed to 4 or 6 phases:
    ///    - **4 Phases**: Acceleration, possibly a constant velocity phase, then deceleration.
    ///    - **6 Phases**: Double acceleration (init→v_max, v_max→exit), where v_max is determined by binary search (`fit_v_max`).
    ///
    /// As a result, either a 4-phase option is found to be shorter, or a 6-phase is needed after all (accelerate to a certain v_max, then decelerate).
    pub fn process(&mut self, const_vel: bool) -> SCurveTask {
        // If SCurve is default (invalid or zero-params), nothing to compute.
        self.task = SCurveTask::default();
        if !self.is_valid() {
            return self.task; // Return zero task
        }

        // 1) Write known parameters to the task executor
        self.task.pos.0 = self.pos.0; // Store init position
        self.task.pos.1 = self.pos.1; // Store exit position
        self.task.vel.0 = self.vel.0; // Store init velocity
        self.task.vel.1 = self.vel.1; // Store exit velocity

        // 2) Compute total distance and direction
        let ds_raw = self.pos.1 - self.pos.0; // Calculate raw distance
        let direction = ds_raw.signum(); // +1.0 or -1.0
        let direction = if direction == 0.0 { 1.0 } else { direction }; // Avoid zero sign
        let ds_abs = ds_raw.abs(); // Work with absolute distance

        // Adjust init/exit velocities to match direction
        let v_init = self.vel.0 * direction; // Make sure velocity sign matches direction
        let v_exit = self.vel.1 * direction; // Make sure velocity sign matches direction

        // Extract and rename local copies for constraints
        let v_lim = self.vel_lim; // Velocity limit
        let a_lim = self.acc_lim; // Acceleration limit
        let j_lim = self.jrk_lim; // Jerk limit

        // Extract jerk components

        let (j_init, j_mid, j_exit) = self.jrk;

        // 3) Try 7-phase motion (full S-curve attempt)
        //    We use calc_six_phase_t_s_a to see if we can accelerate up to v_lim and then slow down.
        let (time_13_57, distance_13_57, (acc_phase1, acc_phase2)) =
            Self::calc_six_phase_t_s_a((v_init, v_exit), self.jrk, v_lim, a_lim);

        // Check if velocity limit is actually reached within ds_abs
        if distance_13_57 <= ds_abs {
            // => 7-phase motion is possible, because we can reach v_lim
            self.task.vel_lim = direction * v_lim; // Save velocity limit with correct sign

            // Setup jerk signs for all mid phases
            self.task.jrk.0 = direction * j_init;
            self.task.jrk.1 = direction * j_mid;
            self.task.jrk.2 = direction * j_mid;
            self.task.jrk.3 = direction * j_exit;

            if const_vel {
                // If constant velocity phase is forced => simply compute leftover distance as constant motion
                let dist4 = ds_abs - distance_13_57; // leftover distance
                let time4 = Self::safe_t(dist4 / v_lim.abs()); // time @ constant velocity

                self.task.acc.0 = direction * acc_phase1; // final acceleration from segment 1
                self.task.acc.1 = direction * acc_phase2; // final acceleration from segment 2
                self.task.duration = time_13_57 + time4; // total motion time
            } else {
                // Otherwise, we do advance fit of j_mid to exactly match ds_abs
                // fit_j_mid uses binary search to find j_mid scale so that total distance == ds_abs
                let (j2_fitted, a1, a2, total_time) = Self::fit_j_mid(
                    ds_abs,
                    (v_init, v_exit),
                    self.jrk,
                    v_lim,
                    a_lim,
                    self.epsilon,
                    Self::BINARY_SEARCH_DEPTH,
                );
                self.task.jrk.1 = direction * j2_fitted; // updated j_mid
                self.task.jrk.2 = direction * j2_fitted; // symmetrical for phase
                self.task.acc.0 = direction * a1; // final accel after initial jerk
                self.task.acc.1 = direction * a2; // final accel before exit jerk
                self.task.duration = total_time; // total time
            }
            return self.task; // 7-phase motion
        }

        // 4) If we got here => 7-phase is impossible (v_lim can't be reached).
        //    Next check if we only need 3-phase motion to shift v_init -> v_exit within ds_abs.
        let (time_3ph, ds_3ph, a_3ph) =
            Self::calc_dv_shift_t_s_a(v_init, v_exit, j_init, j_exit, a_lim);

        if ds_abs <= ds_3ph {
            // => 3 phases are enough (accelerate or decelerate from v_init to v_exit)
            let (j1_f, j3_f, a_max, total_time) = Self::fit_j_init_exit(
                ds_abs,
                (v_init, v_exit),
                (j_init, j_mid, j_exit),
                a_lim,
                j_lim,
                self.epsilon,
                Self::BINARY_SEARCH_DEPTH,
            );

            // Depending on which side we accelerate to/from
            if v_init <= v_exit {
                // Accel from v_init up to v_exit
                self.task.vel_lim = direction * v_exit; // final velocity
                self.task.acc.0 = direction * a_max; // peak acceleration
                self.task.jrk.0 = direction * j1_f; // fitted jerk for init phase
                self.task.jrk.1 = direction * j3_f; // fitted jerk for exit phase
            } else {
                // Decel from v_init down to v_exit
                self.task.vel_lim = direction * v_init;
                self.task.acc.1 = direction * a_max;
                self.task.jrk.2 = direction * j1_f;
                self.task.jrk.3 = direction * j3_f;
            }

            self.task.duration = total_time; // total motion time
            return self.task; // 3-phase motion
        }

        // 5) If we still have leftover distance => either 4 or 6 phases:
        //    4 phases: Accel (or decel), maybe constant velocity, then decel (or accel).
        //    6 phases: Double accel + double decel, but never hitting the official v_lim.

        // 5a) 4-phase: check time if we insert a constant velocity segment
        let s_v_const = ds_abs - ds_3ph; // leftover distance if we do 3-phase for velocity shift
        let t_v_const_init = Self::safe_t(s_v_const / v_init.max(1e-12)); // avoid division by zero
        let t_v_const_exit = Self::safe_t(s_v_const / v_exit.max(1e-12));
        // choose minimal feasible time for constant velocity
        let t_v_const = if t_v_const_init == 0.0 {
            t_v_const_exit
        } else if t_v_const_exit == 0.0 {
            t_v_const_init
        } else {
            t_v_const_init.min(t_v_const_exit)
        };
        let time_4ph = time_3ph + t_v_const; // total time of a "3-phase + constant"

        // 5b) 6-phase: find v_max < v_lim that satisfies ds_abs exactly
        // with two 3-phase combos (v_init→v_max, v_max→v_exit)
        let (v_max, a_init, a_exit, time_6ph) = Self::fit_v_max(
            ds_abs,
            (v_init, v_exit),
            (j_init, j_mid, j_exit),
            v_lim,
            a_lim,
            self.epsilon,
            Self::BINARY_SEARCH_DEPTH,
        );

        // Decide which approach is faster or feasible
        if s_v_const >= 0.0 && time_4ph <= time_6ph && (v_init > 0.0 || v_exit > 0.0) {
            // => 4-phase is viable
            if const_vel {
                // If forced constant velocity:
                if v_init <= v_exit {
                    // Accel from v_init up to v_exit
                    self.task.vel_lim = direction * v_exit;
                    self.task.acc.0 = direction * a_3ph;
                    self.task.jrk.0 = direction * j_init;
                    self.task.jrk.1 = direction * j_exit;
                } else {
                    // Decel from v_init down to v_exit
                    self.task.vel_lim = direction * v_init;
                    self.task.acc.1 = direction * a_3ph;
                    self.task.jrk.2 = direction * j_init;
                    self.task.jrk.3 = direction * j_exit;
                }
                self.task.duration = time_3ph + t_v_const; // total time for 4 phases
                return self.task;
            } else {
                // If not forced constant velocity, we do a small binary search for j_mid
                let (j_mid_adj, a_init, a_exit, time_3ph) = Self::fit_j_mid(
                    ds_abs,
                    (v_init, v_exit),
                    (j_init, j_exit, 0.0),
                    v_exit.max(v_init), // we can't exceed whichever is higher
                    a_lim,
                    self.epsilon,
                    Self::BINARY_SEARCH_DEPTH,
                );
                if v_init <= v_exit {
                    // Accel from v_init up to v_exit
                    self.task.vel_lim = direction * v_exit;
                    self.task.acc.0 = direction * a_init;
                    self.task.jrk.0 = direction * j_init;
                    self.task.jrk.1 = direction * j_mid_adj;
                } else {
                    // Decel from v_init down to v_exit
                    self.task.vel_lim = direction * v_init;
                    self.task.acc.1 = direction * a_exit;
                    self.task.jrk.2 = direction * j_mid_adj;
                    self.task.jrk.3 = direction * j_exit;
                }
                self.task.duration = time_3ph; // total time for that approach

                // If time14 is indeed smaller than time6Ph, pick 4-phase
                if time_3ph < time_6ph && time_3ph > 0.0 {
                    return self.task;
                }
            }
        }

        // 6) If we are here => 6-phase is the final approach
        //    (we cannot do 4-phase faster than 6-phase, or we have negative leftover)
        // Configure jerk and accelerations for final 6-phase scenario
        self.task.jrk.0 = direction * j_init;
        self.task.jrk.1 = direction * j_mid;
        self.task.jrk.2 = direction * j_mid;
        self.task.jrk.3 = direction * j_exit;

        self.task.acc.0 = direction * a_init;
        self.task.acc.1 = direction * a_exit;

        // If initial velocity is bigger than vMax, reverse jerk for the first phase
        if v_init > v_max {
            self.task.jrk.0 *= -1.0;
            self.task.jrk.1 *= -1.0;
            self.task.acc.0 *= -1.0;
        }
        // If final velocity is bigger than vMax, reverse jerk for the second part
        if v_max < v_exit {
            self.task.jrk.2 *= -1.0;
            self.task.jrk.3 *= -1.0;
            self.task.acc.1 *= -1.0;
        }

        self.task.vel_lim = direction * v_max; // new max velocity
        self.task.duration = time_6ph; // total motion time
        self.task // 6-phase motion
    }

    // -----------------------------------------------------------------
    //  Additional "setter" methods for SCurve
    // -----------------------------------------------------------------

    /// Set goal:
    /// `pos: (init: f64, exit: f64)`
    /// `vel: (init: f64, exit: f64)`
    /// `jerk: (init: f64, mid: f64, exit: f64)`
    pub fn set_goal(&mut self, pos: (f64, f64), vel: (f64, f64), jerk: (f64, f64, f64)) -> bool {
        self.is_goal_valid = false;
        if self.is_lim_valid {
            // Check position feasibility within real range
            let pos_start_is_valid = pos.0.is_finite() && pos.0.abs() < Self::P_MAX;
            let pos_exit_is_valid = pos.1.is_finite() && pos.1.abs() < Self::P_MAX;

            // Calculate total distance and sign of movement
            let distance_delta = pos.1 - pos.0; // Delta S
            let distance_sign = distance_delta.signum(); // +1.0 or -1.0

            // Validate velocities to be in [-vel_lim, vel_lim]
            let vel_start_is_valid = vel.0.is_finite() && vel.0.abs() <= self.vel_lim;
            let vel_exit_is_valid = vel.1.is_finite() && vel.1.abs() <= self.vel_lim;

            // Both positions must be valid, both velocities must be valid
            let pos_valid = pos_start_is_valid && pos_exit_is_valid;
            let vel_valid = vel_start_is_valid && vel_exit_is_valid;

            // Check that distance is not too small to require movement
            let is_distance_valid = distance_delta.abs() >= Self::S_MIN;

            // If distance is near zero, we allow a velocity change if initial != final velocity
            let is_motion_feasible =
                pos_valid && vel_valid && (is_distance_valid || vel.0 != vel.1);

            // Minimal distance required to shift from vel.0 to vel.1 with given jerk and acceleration
            let (_, min_distance, _) =
                Self::calc_dv_shift_t_s_a(vel.0, vel.1, self.jrk_lim, self.jrk_lim, self.acc_lim);

            // Check if minimal required distance is <= actual distance
            let is_motion_ok = min_distance * distance_sign <= distance_delta.abs();

            // Clamps each jerk component
            let j_init = Self::safe_clamp(jerk.0.abs(), self.jrk_lim);
            let j_mid = Self::safe_clamp(jerk.1.abs(), self.jrk_lim);
            let j_exit = Self::safe_clamp(jerk.2.abs(), self.jrk_lim);

            if is_motion_feasible && is_motion_ok {
                self.jrk = (j_init, j_mid, j_exit);
                self.pos = pos;
                self.vel = vel;
                self.is_goal_valid = true;
                return true;
            }
        }
        false
    }

    /// Set the three limits: velocity, acceleration, jerk
    /// MUST be called before setting goal
    pub fn set_limits(&mut self, v_lim: f64, a_lim: f64, j_lim: f64) -> bool {
        // Clamp velocity limit to a safe range
        let vel_lim = Self::safe_clamp(v_lim, Self::V_MAX);
        // Clamps acceleration limit
        let acc_lim = Self::safe_clamp(a_lim, Self::A_MAX);
        // Clamps jerk limit
        let jrk_lim = Self::safe_clamp(j_lim, Self::J_MAX);

        // If at least one of jerk or acceleration is not saturating, we consider them valid
        if jrk_lim == Self::J_MAX && acc_lim == Self::A_MAX {
            self.is_lim_valid = false;
        } else {
            self.is_lim_valid = true;
            self.vel_lim = vel_lim;
            self.acc_lim = acc_lim;
            self.jrk_lim = jrk_lim;
        }
        self.is_lim_valid
    }

    // -----------------------------------------------------------------
    //  Getter methods for SCurve
    // -----------------------------------------------------------------

    /// Get current positions `(pos_init, pos_exit)`.
    pub fn get_pos(&self) -> (f64, f64) {
        self.pos
    }

    /// Get current velocities `(vel_init, vel_exit)`.
    pub fn get_vel(&self) -> (f64, f64) {
        self.vel
    }

    /// Get current jerk components `(j_init, j_mid, j_exit)`.
    pub fn get_jrk(&self) -> (f64, f64, f64) {
        self.jrk
    }

    /// Get current velocity limit.
    pub fn get_vel_lim(&self) -> f64 {
        self.vel_lim
    }

    /// Get current acceleration limit.
    pub fn get_acc_lim(&self) -> f64 {
        self.acc_lim
    }

    /// Get current jerk limit.
    pub fn get_jrk_lim(&self) -> f64 {
        self.jrk_lim
    }

    /// Get current SCurve task
    pub fn get_task(&self) -> SCurveTask {
        self.task
    }

    // -----------------------------------------------------------------------------------------
    // Below are helper methods for internal calculations:
    // -----------------------------------------------------------------------------------------

    /// Binary search to fit jerk during init/exit phase (3-phase approach),
    /// ensuring we match the target distance exactly when shifting v_init→v_exit.
    fn fit_j_init_exit(
        ds: f64,       // total distance
        v: (f64, f64), // (init_velocity, exit_velocity)
        j: (f64, f64, f64),
        a_lim: f64,   // acceleration limit
        jrk_lim: f64, // jerk limit
        epsilon: f64, // binary search precision
        max_iterations: usize,
    ) -> (f64, f64, f64, f64) {
        let (v_init, v_exit) = v; // extract velocities
        let (j_init, _, j_exit) = j; // extract jerk components

        // Initial bracket for scale
        let mut scale_min = 1.0; // minimal scale
        let mut scale_max = 1.0; // maximal scale
        let mut s_calc ; // distance placeholder

        // Exponential search to find upper bound
        for _ in 0..max_iterations {
            scale_max *= 2.0; // expand scale
            let j_init_adj = (j_init * scale_max).min(jrk_lim);
            let j_exit_adj = (j_exit * scale_max).min(jrk_lim);
            let (_, s_calc_tmp, _) =
                Self::calc_dv_shift_t_s_a(v_init, v_exit, j_init_adj, j_exit_adj, a_lim);
            s_calc = s_calc_tmp;
            if s_calc < ds {
                break; // found bracket
            }
        }

        // Now binary search in [scale_min, scale_max]
        let mut t_final = 0.0; // store final time
        let mut a_final = a_lim; // store final acceleration
        let (mut j_init_adj, mut j_exit_adj) = (jrk_lim, jrk_lim);

        for _ in 0..max_iterations {
            let scale = (scale_min + scale_max) / 2.0;
            j_init_adj = (j_init * scale).min(jrk_lim);
            j_exit_adj = (j_exit * scale).min(jrk_lim);

            let (t_calc, s_calc_tmp, a_calc) =
                Self::calc_dv_shift_t_s_a(v_init, v_exit, j_init_adj, j_exit_adj, a_lim);
            t_final = t_calc;
            s_calc = s_calc_tmp;
            a_final = a_calc;

            // check closeness
            let diff = (s_calc - ds).abs();
            if diff <= epsilon {
                break;
            } else if s_calc > ds {
                scale_min = scale;
            } else {
                scale_max = scale;
            }
        }
        (j_init_adj, j_exit_adj, a_final, t_final)
    }

    /// Binary search to find v_max that satisfies distance ds for a 6-phase approach.
    fn fit_v_max(
        ds: f64,            // total distance
        v: (f64, f64),      // (init_velocity, exit_velocity)
        j: (f64, f64, f64), // (j_init, j_mid, j_exit)
        v_lim: f64,         // velocity limit
        a_lim: f64,         // acceleration limit
        epsilon: f64,       // binary search precision
        max_iterations: usize,
    ) -> (f64, f64, f64, f64) {
        // We pick up to three candidate ranges for v_max around [-v_lim, v_lim].
        // This is a heuristic approach to bracket solutions from below and above.
        let mut v_min = [-v_lim, v.0.min(v.1), v.0.max(v.1)];
        let mut v_max = [v.0.min(v.1), v.0.max(v.1), v_lim];

        // Arrays to hold intermediate results
        let mut v_find = [0.0, 0.0, 0.0];
        let mut s_calc = [0.0, 0.0, 0.0];
        let mut a1 = [a_lim, a_lim, a_lim];
        let mut a2 = [a_lim, a_lim, a_lim];
        let mut t_calc = [0.0, 0.0, 0.0];
        let mut converged = [false, false, false];

        for i in 0..3 {
            for _ in 0..max_iterations {
                // midpoint
                let v_candidate = (v_min[i] + v_max[i]) / 2.0;
                let (t_tmp, s_tmp, (acc1, acc2)) =
                    Self::calc_six_phase_t_s_a(v, j, v_candidate, a_lim);
                v_find[i] = v_candidate;
                t_calc[i] = t_tmp;
                s_calc[i] = s_tmp;
                a1[i] = acc1;
                a2[i] = acc2;

                if (s_tmp - ds).abs() <= epsilon {
                    converged[i] = true;
                    break;
                } else if s_tmp < ds {
                    v_min[i] = v_candidate;
                } else {
                    v_max[i] = v_candidate;
                }
            }
        }

        // Choose the best candidate among possible intervals
        let best_index = if converged.iter().any(|&x| x) {
            // If one or more indexes converged to epsilon, pick the minimal time among them
            converged
                .iter()
                .enumerate()
                .filter(|&(_, &c)| c)
                .min_by(|(i1, _), (i2, _)| t_calc[*i1].partial_cmp(&t_calc[*i2]).unwrap())
                .map(|(idx, _)| idx)
                .unwrap()
        } else {
            // If none converged, pick the solution that yields distance closest to ds
            s_calc
                .iter()
                .enumerate()
                .min_by(|(_, dist_a), (_, dist_b)| {
                    (*dist_a - ds)
                        .abs()
                        .partial_cmp(&(*dist_b - ds).abs())
                        .unwrap()
                })
                .map(|(idx, _)| idx)
                .unwrap()
        };

        // Return found v_max, final accelerations, and total time
        (
            v_find[best_index],
            a1[best_index],
            a2[best_index],
            t_calc[best_index],
        )
    }

    /// Binary search for fitting j_mid to match the exact distance for a 6-phase approach
    /// if we want to carefully refine the "middle jerk" portion.
    fn fit_j_mid(
        ds: f64,            // total distance
        v: (f64, f64),      // (start_velocity, end_velocity)
        j: (f64, f64, f64), // (j_init, j_mid, j_exit)
        v_lim: f64,         // velocity limit
        a_lim: f64,         // acceleration limit
        epsilon: f64,       // binary search precision
        max_iterations: usize,
    ) -> (f64, f64, f64, f64) {
        let j_mid_original = j.1; // original j_mid
        let mut j_mid_adjusted = j_mid_original;

        // For the scale factor
        let mut scale_min = 0.0;
        let mut scale_max = 1.0;
        let mut dist_temp;

        // placeholders for final results
        let mut total_time = 0.0;
        let mut a1_final = a_lim;
        let mut a2_final = a_lim;

        for _ in 0..max_iterations {
            // take the average scale
            let scale = (scale_min + scale_max) / 2.0;
            j_mid_adjusted = j_mid_original * scale;

            // calculate total time/distance with updated j_mid
            let (t_calc, s_calc, (a1_calc, a2_calc)) =
                Self::calc_six_phase_t_s_a((v.0, v.1), (j.0, j_mid_adjusted, j.2), v_lim, a_lim);

            dist_temp = s_calc;
            total_time = t_calc;
            a1_final = a1_calc;
            a2_final = a2_calc;

            // check difference
            let diff = (dist_temp - ds).abs();
            if diff <= epsilon {
                break; // close enough
            } else if dist_temp > ds {
                scale_min = scale;
            } else {
                scale_max = scale;
            }
        }
        (j_mid_adjusted, a1_final, a2_final, total_time)
    }

    /// Calculates times and distances in a 6-phase approach:
    /// (accel from v_init to v_lim) + (accel from v_lim to v_exit)
    /// using j_init (phase1), j_mid (phase2), j_exit (phase3).
    /// Essentially splits the velocity change into two 3-phase calculations.
    fn calc_six_phase_t_s_a(
        v: (f64, f64),      // (start_velocity, end_velocity)
        j: (f64, f64, f64), // (j_init, j_mid, j_exit)
        v_lim: f64,         // velocity limit
        a_lim: f64,         // acceleration limit
    ) -> (f64, f64, (f64, f64)) {
        // 1) from v_init to v_lim
        let (t1, s1, a1) = Self::calc_dv_shift_t_s_a(v.0, v_lim, j.0, j.1, a_lim);
        // 2) from v_lim to v.1
        let (t2, s2, a2) = Self::calc_dv_shift_t_s_a(v_lim, v.1, j.1, j.2, a_lim);

        let total_time = t1 + t2; // sum of times
        let total_distance = s1 + s2; // sum of distances
        (total_time, total_distance, (a1, a2))
    }

    /// Calculates the minimal time and distance to shift from v_init to v_exit with
    /// given positive jerks j_init, j_exit, abiding by acceleration limit.
    /// This is the "3-phase" approach:
    ///  Phase1: Jerk up or down from v_init
    ///  Phase2: Possibly hold max acceleration (if needed)
    ///  Phase3: Jerk down or up to reach v_exit
    fn calc_dv_shift_t_s_a(
        v_init: f64,  // initial velocity
        v_exit: f64,  // target velocity
        j_init: f64,  // jerk for initial phase
        j_exit: f64,  // jerk for exit phase
        acc_lim: f64, // max acceleration
    ) -> (f64, f64, f64) {
        // If no velocity change is needed, return zero times/distance
        if (v_init - v_exit).abs() < f64::EPSILON {
            return (0.0, 0.0, 0.0);
        }

        // 1) Compute velocity difference
        let dv = (v_init - v_exit).abs();

        // 2) Compute times for initial and exit jerks (assuming symmetrical jerk usage)
        let mut t_j_init = ((2.0 * dv * j_exit) / (j_init * (j_init + j_exit))).sqrt();
        t_j_init = Self::safe_t(t_j_init);

        let mut t_j_exit;
        if t_j_init == 0.0 {
            // fallback formula if above fails (dv might be small or j_init near zero)
            t_j_exit = (2.0 * dv / j_exit).sqrt();
            t_j_exit = Self::safe_t(t_j_exit);
        } else {
            t_j_exit = ((2.0 * dv * j_init) / (j_exit * (j_init + j_exit))).sqrt();
            t_j_exit = Self::safe_t(t_j_exit);
            if t_j_exit == 0.0 {
                // fallback if second sqrt is zero => use symmetrical approach
                t_j_init = (2.0 * dv / j_init).sqrt();
                t_j_init = Self::safe_t(t_j_init);
            }
        }

        // 3) Compute maximum acceleration reached after phases 1 & 3
        //    It's roughly a trapezoid shape in the velocity-time diagram
        let mut a_max = if t_j_init == 0.0 && t_j_exit == 0.0 {
            acc_lim
        } else if t_j_init == 0.0 {
            j_exit * t_j_exit
        } else if t_j_exit == 0.0 {
            j_init * t_j_init
        } else {
            0.5 * (j_exit * t_j_exit + j_init * t_j_init)
        };

        // 4) If that maximum acceleration surpasses acc_lim, clamp it,
        //    and insert a constant-acceleration phase if needed
        let mut t_a_const = 0.0;
        if a_max >= acc_lim {
            t_j_init = Self::safe_t(acc_lim / j_init);
            t_j_exit = Self::safe_t(acc_lim / j_exit);
            let dv_init = 0.5 * j_init * t_j_init.powi(2);
            let dv_exit = 0.5 * j_exit * t_j_exit.powi(2);
            t_a_const = Self::safe_t((dv - dv_init - dv_exit).abs() / acc_lim);
            a_max = acc_lim;
        }

        // 5) Compute partial velocity increments to get the distance
        let sign = (v_exit - v_init).signum(); // +1 or -1
        let dv_init = 0.5 * j_init * t_j_init.powi(2);
        let dv_exit = 0.5 * j_exit * t_j_exit.powi(2);

        // 6) Distances in each phase
        //    Phase1: from v_init to end of jerk1
        let ds_j_init = (v_init + sign * dv_init / 3.0) * t_j_init;
        //    Phase2: possible constant acceleration phase
        let ds_a_const = (v_init + sign * dv_init + sign * 0.5 * a_max * t_a_const) * t_a_const;
        //    Phase3: from start of jerk2 to v_exit
        let ds_j_exit = (v_exit - sign * (dv_exit / 3.0)) * t_j_exit;

        // 7) Summation of times and distances
        let total_time = t_j_init + t_a_const + t_j_exit;
        let total_distance = ds_j_init + ds_a_const + ds_j_exit;

        (total_time, total_distance, a_max)
    }

    /// Clamps a given value to [value.abs() ... max], ensures it is finite and not zero.
    fn safe_clamp(value: f64, max: f64) -> f64 {
        // Take absolute value to keep sign logic consistent
        let val_abs = value.abs();
        if !val_abs.is_finite() || val_abs <= 1e-40 || val_abs >= max {
            // If it's infinite, zero, or exceeds max, clamp to max
            max
        } else {
            val_abs
        }
    }

    /// Ensures time `t` is within [0.0 ... T_MAX], or zero if < T_MIN
    fn safe_t(t: f64) -> f64 {
        if t >= Self::T_MAX {
            Self::T_MAX
        } else if t >= Self::T_MIN {
            t
        } else {
            0.0
        }
    }
}