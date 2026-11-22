//! asv-core — C-ABI compatible Rust implementation of core control & math.
//! Tabel & aturan disamakan persis dengan core/src/*.c agar output bit-identik.
#![allow(unsafe_code)]
#![allow(non_snake_case)]

use std::f64::consts::PI;

pub const NAV_EARTH_RADIUS_M: f64 = 6371000.0;

// ---------------------------------------------------------------------------
// Nav Math
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn nav_normalize_angle(mut angle_rad: f64) -> f64 {
    while angle_rad > PI {
        angle_rad -= 2.0 * PI;
    }
    while angle_rad < -PI {
        angle_rad += 2.0 * PI;
    }
    angle_rad
}

#[no_mangle]
pub unsafe extern "C" fn nav_distance_bearing(
    lat1: f64,
    lon1: f64,
    lat2: f64,
    lon2: f64,
    bearing_out: *mut f64,
) -> f64 {
    if !bearing_out.is_null() {
        *bearing_out = 0.0;
    }
    if lat1.is_nan() || lon1.is_nan() || lat2.is_nan() || lon2.is_nan() {
        return f64::INFINITY;
    }

    let phi1 = lat1 * (PI / 180.0);
    let phi2 = lat2 * (PI / 180.0);
    let d_phi = (lat2 - lat1) * (PI / 180.0);
    let d_lambda = (lon2 - lon1) * (PI / 180.0);

    let mut a = (d_phi / 2.0).sin().powi(2)
        + phi1.cos() * phi2.cos() * (d_lambda / 2.0).sin().powi(2);
    if a < 0.0 { a = 0.0; }
    if a > 1.0 { a = 1.0; }

    let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());
    let distance = NAV_EARTH_RADIUS_M * c;

    let y = d_lambda.sin() * phi2.cos();
    let x = phi1.cos() * phi2.sin() - phi1.sin() * phi2.cos() * d_lambda.cos();
    let bearing = y.atan2(x);

    if !bearing_out.is_null() {
        *bearing_out = bearing;
    }
    distance
}

#[no_mangle]
pub unsafe extern "C" fn nav_destination(
    lat: f64,
    lon: f64,
    distance_m: f64,
    bearing_rad: f64,
    lat_out: *mut f64,
    lon_out: *mut f64,
) {
    let lat1 = lat * (PI / 180.0);
    let lon1 = lon * (PI / 180.0);
    let angular = distance_m / NAV_EARTH_RADIUS_M;

    let lat2 = (lat1.sin() * angular.cos() + lat1.cos() * angular.sin() * bearing_rad.cos()).asin();
    let lon2 = lon1 + (bearing_rad.sin() * angular.sin() * lat1.cos())
        .atan2(angular.cos() - lat1.sin() * lat2.sin());

    if !lat_out.is_null() { *lat_out = lat2 * (180.0 / PI); }
    if !lon_out.is_null() { *lon_out = lon2 * (180.0 / PI); }
}

#[no_mangle]
pub unsafe extern "C" fn nav_cross_track_distance(
    lat_p: f64, lon_p: f64,
    lat_wp1: f64, lon_wp1: f64,
    lat_wp2: f64, lon_wp2: f64,
) -> f64 {
    let mut brg_1p: f64 = 0.0;
    let mut brg_12: f64 = 0.0;
    let dist_1p = nav_distance_bearing(lat_wp1, lon_wp1, lat_p, lon_p, &mut brg_1p);
    nav_distance_bearing(lat_wp1, lon_wp1, lat_wp2, lon_wp2, &mut brg_12);

    if dist_1p.is_infinite() || dist_1p == 0.0 { return 0.0; }

    let mut term = dist_1p / NAV_EARTH_RADIUS_M;
    if term > PI { term = PI; }
    if term < -PI { term = -PI; }

    let mut sin_term = term.sin() * (brg_1p - brg_12).sin();
    if sin_term > 1.0 { sin_term = 1.0; }
    if sin_term < -1.0 { sin_term = -1.0; }

    sin_term.asin() * NAV_EARTH_RADIUS_M
}

// ---------------------------------------------------------------------------
// Control Filters (PID & EKF)
// ---------------------------------------------------------------------------

#[repr(C)]
pub struct cf_pid_t {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub deadband: f64,
    pub output_limit: f64,
    pub integral_limit: f64,
    pub integral: f64,
    pub last_error: f64,
}

const CF_MIN_DT: f64 = 1e-4;

#[no_mangle]
pub unsafe extern "C" fn cf_pid_reset(p: *mut cf_pid_t) {
    if let Some(p) = p.as_mut() {
        p.integral = 0.0;
        p.last_error = 0.0;
    }
}

#[no_mangle]
pub unsafe extern "C" fn cf_pid_update(p: *mut cf_pid_t, error: f64, mut dt: f64) -> f64 {
    let p = match p.as_mut() { Some(ptr) => ptr, None => return 0.0 };
    if dt <= 0.0 { dt = CF_MIN_DT; }
    if error < p.deadband && error > -p.deadband {
        p.last_error = error;
        return 0.0;
    }
    p.integral += error * dt;
    if p.integral_limit > 0.0 {
        p.integral = p.integral.clamp(-p.integral_limit, p.integral_limit);
    }
    let derivative = (error - p.last_error) / dt;
    p.last_error = error;

    let mut output = p.kp * error + p.ki * p.integral + p.kd * derivative;
    if p.output_limit > 0.0 {
        output = output.clamp(-p.output_limit, p.output_limit);
    }
    output
}

#[no_mangle]
pub unsafe extern "C" fn cf_complementary(
    alpha: f64, angle_prev: f64, gyro_rate: f64, mut dt: f64, angle_measured: f64,
) -> f64 {
    if dt <= 0.0 { dt = CF_MIN_DT; }
    alpha * (angle_prev + gyro_rate * dt) + (1.0 - alpha) * angle_measured
}

#[repr(C)]
pub struct cf_ekf_t {
    pub q_heading: f64,
    pub q_bias: f64,
    pub r_meas: f64,
    pub heading: f64,
    pub bias: f64,
    pub p00: f64,
    pub p01: f64,
    pub p11: f64,
}

#[no_mangle]
pub unsafe extern "C" fn cf_ekf_init(
    e: *mut cf_ekf_t, process_noise: f64, meas_noise: f64, init_heading: f64,
) {
    if let Some(e) = e.as_mut() {
        e.q_heading = process_noise;
        e.q_bias = process_noise * 0.1;
        e.r_meas = meas_noise;
        e.heading = init_heading;
        e.bias = 0.0;
        e.p00 = 1.0; e.p01 = 0.0; e.p11 = 1.0;
    }
}

#[no_mangle]
pub unsafe extern "C" fn cf_ekf_predict(e: *mut cf_ekf_t, gyro_rate: f64, mut dt: f64) -> f64 {
    let e = match e.as_mut() { Some(ptr) => ptr, None => return 0.0 };
    if dt <= 0.0 { dt = CF_MIN_DT; }
    e.heading += (gyro_rate - e.bias) * dt;
    let p00 = e.p00 - 2.0 * dt * e.p01 + dt * dt * e.p11 + e.q_heading * dt;
    let p01 = e.p01 - dt * e.p11;
    let p11 = e.p11 + e.q_bias * dt;
    e.p00 = p00; e.p01 = p01; e.p11 = p11;
    e.heading
}

#[no_mangle]
pub unsafe extern "C" fn cf_ekf_update(e: *mut cf_ekf_t, measured_heading: f64) -> f64 {
    let e = match e.as_mut() { Some(ptr) => ptr, None => return 0.0 };
    let innovation = nav_normalize_angle(measured_heading - e.heading);
    let mut s = e.p00 + e.r_meas;
    if s <= 0.0 { s = CF_MIN_DT; }
    let k0 = e.p00 / s;
    let k1 = e.p01 / s;
    e.heading += k0 * innovation;
    e.bias += k1 * innovation;
    let p00 = e.p00 - k0 * e.p00;
    let p01 = e.p01 - k0 * e.p01;
    let p11 = e.p11 - k1 * e.p01;
    e.p00 = p00; e.p01 = p01; e.p11 = p11;
    nav_normalize_angle(e.heading)
}

#[no_mangle]
pub unsafe extern "C" fn cf_ekf_heading(e: *const cf_ekf_t) -> f64 {
    match e.as_ref() { Some(e) => nav_normalize_angle(e.heading), None => 0.0 }
}

// ---------------------------------------------------------------------------
// Gate Vision helper
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn gv_estimate_distance(
    pixel_width: f64, gate_width_m: f64, focal_length_px: f64,
) -> f64 {
    if pixel_width <= 5.0 || focal_length_px <= 0.0 || gate_width_m <= 0.0 {
        f64::INFINITY
    } else {
        (gate_width_m * focal_length_px) / pixel_width
    }
}

#[no_mangle]
pub unsafe extern "C" fn gv_yaw_correction(
    target_x_px: f64, image_center_x: f64,
    distance_m: f64, focal_length_px: f64,
) -> f64 {
    if distance_m < 0.1 || focal_length_px <= 0.0 {
        0.0
    } else {
        let dx_m = ((target_x_px - image_center_x) * distance_m) / focal_length_px;
        dx_m.atan2(distance_m)
    }
}

// ---------------------------------------------------------------------------
// Thruster Mixer
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn mixer_clamp(val: f64, lo: f64, hi: f64) -> f64 {
    val.clamp(lo, hi)
}

#[no_mangle]
pub unsafe extern "C" fn mixer_diff_drive(
    surge: f64, yaw: f64,
    pwm_left: *mut i32, pwm_right: *mut i32,
) {
    let surge = surge.clamp(-1.0, 1.0);
    let yaw = yaw.clamp(-1.0, 1.0);
    let span = 400.0;
    let left_us = (1500.0 + surge * span - yaw * span).clamp(1100.0, 1900.0);
    let right_us = (1500.0 + surge * span + yaw * span).clamp(1100.0, 1900.0);
    if !pwm_left.is_null() { *pwm_left = (left_us + 0.5) as i32; }
    if !pwm_right.is_null() { *pwm_right = (right_us + 0.5) as i32; }
}

// ---------------------------------------------------------------------------
// Manual arbitrator (QGC vs lokal)
// ---------------------------------------------------------------------------

#[repr(C)]
pub struct arb_manual_t {
    pub surge: f64,
    pub yaw: f64,
    pub source: i32,
    pub conflict: i32,
}

#[no_mangle]
pub unsafe extern "C" fn arb_manual2(
    qgc_active: i32, qgc_surge: f64, qgc_yaw: f64,
    loc_active: i32, loc_surge: f64, loc_yaw: f64,
    out: *mut arb_manual_t,
) {
    let out = match out.as_mut() { Some(ptr) => ptr, None => return };
    if qgc_active != 0 && loc_active != 0 {
        out.surge = 0.0; out.yaw = 0.0; out.source = 3; out.conflict = 1;
    } else if qgc_active != 0 {
        out.surge = qgc_surge.clamp(-1.0, 1.0);
        out.yaw = qgc_yaw.clamp(-1.0, 1.0);
        out.source = 1; out.conflict = 0;
    } else if loc_active != 0 {
        out.surge = loc_surge.clamp(-1.0, 1.0);
        out.yaw = loc_yaw.clamp(-1.0, 1.0);
        out.source = 2; out.conflict = 0;
    } else {
        out.surge = 0.0; out.yaw = 0.0; out.source = 0; out.conflict = 0;
    }
}

// ---------------------------------------------------------------------------
// Fuzzy Singleton (port 1:1 core/src/fuzzy.c)
// ---------------------------------------------------------------------------

fn fuzzy_trapmf(x: f64, a: f64, b: f64, c: f64, d: f64) -> f64 {
    if b > a {
        if x <= a { return 0.0; }
        if x < b { return (x - a) / (b - a); }
    }
    if d > c && x > c {
        if x >= d { return 0.0; }
        return (d - x) / (d - c);
    }
    1.0
}

type Rule = (i32, i32, i32, f64); // dist_term, has_err, err_term, singleton

fn sugeno_eval(
    dist: f64, error: f64,
    dist_mf: &[[f64; 4]; 3], err_mf: &[[f64; 4]; 3],
    rules: &[Rule],
) -> f64 {
    let mut total_alpha = 0.0;
    let mut total_out = 0.0;

    for &(d_term, has_err, e_term, singleton) in rules {
        let mf_d = dist_mf[d_term as usize];
        let mut alpha = fuzzy_trapmf(dist, mf_d[0], mf_d[1], mf_d[2], mf_d[3]);
        if has_err == 1 {
            let mf_e = err_mf[e_term as usize];
            let alpha_err = fuzzy_trapmf(error, mf_e[0], mf_e[1], mf_e[2], mf_e[3]);
            if alpha_err < alpha { alpha = alpha_err; }
        }
        total_alpha += alpha;
        total_out += alpha * singleton;
    }

    if total_alpha <= 0.0 { 0.0 } else { total_out / total_alpha }
}

const GATE_DIST_MF: [[f64; 4]; 3] = [
    [0.0, 0.0, 0.2, 0.4],   // DEKAT
    [0.3, 0.5, 0.7, 0.8],   // SEDANG
    [0.7, 0.8, 1.0, 1.0],   // JAUH
];
const GATE_ERR_MF: [[f64; 4]; 3] = [
    [0.0, 0.0, 20.0, 40.0],     // KECIL
    [30.0, 60.0, 100.0, 130.0], // SEDANG
    [110.0, 150.0, 320.0, 320.0], // BESAR
];
const GATE_RULES: [Rule; 9] = [
    (0, 1, 0, 1.0),
    (0, 1, 1, 1.9),
    (0, 1, 2, 2.5),
    (1, 1, 0, 1.0),
    (1, 1, 1, 1.9),
    (1, 1, 2, 2.5),
    (2, 1, 0, 1.9),
    (2, 1, 1, 2.5),
    (2, 1, 2, 2.5),
];

#[no_mangle]
pub unsafe extern "C" fn fuzzy_gate_p_gain(jarak_m: f64, error_px: f64) -> f64 {
    sugeno_eval(jarak_m, error_px, &GATE_DIST_MF, &GATE_ERR_MF, &GATE_RULES)
}

const DOCK_DIST_MF: [[f64; 4]; 3] = [
    [0.05, 0.08, 0.12, 0.15], // DEKAT
    [0.13, 0.18, 0.25, 0.35], // SEDANG
    [0.30, 0.40, 1.0, 1.0],   // JAUH
];
const DOCK_ERR_MF: [[f64; 4]; 3] = [
    [0.0, 0.0, 15.0, 30.0],      // KECIL
    [25.0, 50.0, 80.0, 100.0],   // SEDANG
    [90.0, 120.0, 320.0, 320.0], // BESAR
];
const DOCK_RULES: [Rule; 7] = [
    (0, 0, 0, 0.5), // D_DEKAT, no error
    (1, 1, 0, 0.5),
    (1, 1, 1, 1.2),
    (1, 1, 2, 1.9),
    (2, 1, 0, 1.2),
    (2, 1, 1, 1.9),
    (2, 1, 2, 1.9),
];

#[no_mangle]
pub unsafe extern "C" fn fuzzy_docking_p_gain(jarak_m: f64, error_px: f64) -> f64 {
    sugeno_eval(jarak_m, error_px, &DOCK_DIST_MF, &DOCK_ERR_MF, &DOCK_RULES)
}
