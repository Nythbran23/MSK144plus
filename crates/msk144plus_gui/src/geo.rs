// crates/msk144plus_gui/src/geo.rs
//
// Geographic helpers for the QSO panel: Maidenhead-locator → lat/lon,
// great-circle distance, true bearing, and meteor-scatter geometry
// (arc bounds + optimal beam headings given the antenna H-beamwidth).
//
// Ported from FSK441Plus's `geo.rs` and `scatter.rs`. Maidenhead
// resolution: when only 4 chars are given, returns the centre of the
// square (per Roger's spec: "we only use 4-figure maidenheads then
// calculate to the centre of the squares"). 6-char locators are
// honoured if present and resolve to subsquare centre.

const R_EARTH:    f64 = 6_371.0;  // km
const SHELL_KM:   f64 = 110.0;    // underdense E-layer for 2 m MS
const MIN_EL_DEG: f64 = 1.0;      // minimum elevation accepted (°)
const AZ_STEP:    f64 = 0.1;      // azimuth sweep resolution (°)

/// A station's QTH, parsed from a Maidenhead locator.
#[derive(Debug, Clone, Copy)]
pub struct Qth {
    pub lat: f64,  // degrees North
    pub lon: f64,  // degrees East (negative = West)
}

impl Qth {
    /// Parse a Maidenhead locator (4 or 6 chars) into lat/lon. Returns
    /// the centre of the square for 4-char input, centre of subsquare
    /// for 6-char.
    pub fn from_maidenhead(grid: &str) -> Option<Self> {
        let g = grid.trim().to_uppercase();
        let b = g.as_bytes();
        if b.len() < 4 {
            return None;
        }
        // Field (2 letters)
        if !b[0].is_ascii_alphabetic() || !b[1].is_ascii_alphabetic() {
            return None;
        }
        // Square (2 digits)
        if !b[2].is_ascii_digit() || !b[3].is_ascii_digit() {
            return None;
        }
        let lon = (b[0].wrapping_sub(b'A')) as f64 * 20.0 - 180.0;
        let lat = (b[1].wrapping_sub(b'A')) as f64 * 10.0 - 90.0;
        let lon = lon + (b[2].wrapping_sub(b'0')) as f64 * 2.0;
        let lat = lat + (b[3].wrapping_sub(b'0')) as f64;

        // Optional subsquare (2 letters)
        let (lon, lat) = if b.len() >= 6
            && b[4].is_ascii_alphabetic()
            && b[5].is_ascii_alphabetic()
        {
            let lo = lon + (b[4].wrapping_sub(b'A')) as f64 * (2.0 / 24.0);
            let la = lat + (b[5].wrapping_sub(b'A')) as f64 * (1.0 / 24.0);
            // Centre of subsquare
            (lo + 1.0 / 24.0, la + 0.5 / 24.0)
        } else {
            // Centre of square (2° × 1°)
            (lon + 1.0, lat + 0.5)
        };
        Some(Qth { lat, lon })
    }
}

/// Great-circle distance in km between two points (Haversine).
pub fn gc_distance_km(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let dlat = (lat2 - lat1).to_radians();
    let dlon = (lon2 - lon1).to_radians();
    let a = (dlat / 2.0).sin().powi(2)
        + lat1.to_radians().cos() * lat2.to_radians().cos()
            * (dlon / 2.0).sin().powi(2);
    R_EARTH * 2.0 * a.sqrt().asin()
}

/// Initial true bearing (°, 0..360) from point 1 to point 2.
pub fn gc_bearing_deg(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let dlon = (lon2 - lon1).to_radians();
    let lat1r = lat1.to_radians();
    let lat2r = lat2.to_radians();
    let y = dlon.sin() * lat2r.cos();
    let x = lat1r.cos() * lat2r.sin() - lat1r.sin() * lat2r.cos() * dlon.cos();
    (y.atan2(x).to_degrees() + 360.0) % 360.0
}

/// Great-circle destination given start lat/lon, bearing (°), distance (km).
fn destination(lat_deg: f64, lon_deg: f64, bearing_deg: f64, dist_km: f64) -> (f64, f64) {
    let lat = lat_deg.to_radians();
    let lon = lon_deg.to_radians();
    let brg = bearing_deg.to_radians();
    let d = dist_km / R_EARTH;
    let lat2 = (lat.sin() * d.cos() + lat.cos() * d.sin() * brg.cos()).asin();
    let lon2 = lon
        + (brg.sin() * d.sin() * lat.cos())
            .atan2(d.cos() - lat.sin() * lat2.sin());
    (lat2.to_degrees(), lon2.to_degrees())
}

/// Elevation angle (°) from station A on the ground to a scatter point P
/// at altitude alt_km. Positive = above horizon. Spherical earth, law of
/// cosines on the triangle (earth-centre, A, P).
fn elevation_angle(
    lat_a: f64, lon_a: f64,
    lat_p: f64, lon_p: f64,
    alt_km: f64,
) -> f64 {
    let d = gc_distance_km(lat_a, lon_a, lat_p, lon_p);
    if d < 0.1 {
        return 90.0;
    }
    let rh = R_EARTH + alt_km;
    let central = d / R_EARTH;
    let ap2 = R_EARTH.powi(2) + rh.powi(2) - 2.0 * R_EARTH * rh * central.cos();
    if ap2 <= 0.0 {
        return 90.0;
    }
    let ap = ap2.sqrt();
    let cos_oap = (R_EARTH.powi(2) + ap2 - rh.powi(2)) / (2.0 * R_EARTH * ap);
    let cos_oap = cos_oap.clamp(-1.0, 1.0);
    cos_oap.acos().to_degrees() - 90.0
}

/// Lat/lon of the scatter point on the SHELL_KM shell that is at
/// (az_deg, MIN_EL_DEG) from station A. Solves the slant-range
/// quadratic, then projects back to the surface using the central
/// angle.
fn shell_point_at_az(lat_a: f64, lon_a: f64, az_deg: f64) -> Option<(f64, f64)> {
    let el = MIN_EL_DEG.to_radians();
    let rh = R_EARTH + SHELL_KM;
    let b = 2.0 * R_EARTH * el.sin();
    let c = -(rh.powi(2) - R_EARTH.powi(2));
    let disc = b * b - 4.0 * c;
    if disc < 0.0 {
        return None;
    }
    let r_slant = (-b + disc.sqrt()) / 2.0;
    if r_slant <= 0.0 {
        return None;
    }
    let sin_angle_p = (R_EARTH * el.cos() / rh).clamp(-1.0, 1.0);
    let angle_p = sin_angle_p.asin();
    let central = std::f64::consts::FRAC_PI_2 - el - angle_p;
    if central < 0.0 {
        return None;
    }
    let ground_dist = R_EARTH * central;
    Some(destination(lat_a, lon_a, az_deg, ground_dist))
}

/// Result of scatter-arc computation.
#[derive(Debug, Clone, Copy)]
pub struct ScatterArc {
    pub gc_bearing: f64,
    pub gc_distance_km: f64,
    /// Leftmost (CCW) bearing in the mutually-visible scatter zone (°)
    pub arc_min: f64,
    /// Rightmost (CW) bearing in the mutually-visible scatter zone (°)
    pub arc_max: f64,
    /// Half-width of the arc (°) — symmetric offset either side of GC
    pub arc_half_width: f64,
    /// Elevation to GC midpoint scatter point (°)
    pub midpoint_el: f64,
    /// Optimal A beam heading (°) — left edge inset by half-beamwidth
    pub beam_a: Option<f64>,
    /// Optimal B beam heading (°) — right edge inset by half-beamwidth
    pub beam_b: Option<f64>,
}

/// Compute scatter arc and optimal A/B beam headings between A and B.
///
/// Sweeps all azimuths from A at MIN_EL_DEG to the SHELL_KM shell;
/// keeps those where B also has elevation ≥ MIN_EL_DEG to that shell
/// point. The resulting arc is the set of bearings from A that land
/// at meteor-burst points mutually visible from both stations.
///
/// `bw_horiz_deg` — antenna 3-dB horizontal beamwidth. Used to inset
/// the two optimal beam headings from the arc edges so the main lobe
/// stays inside the scatter zone. If the arc is narrower than the
/// beamwidth, only one centred heading is returned.
pub fn compute_scatter_arc(
    lat_a: f64, lon_a: f64,
    lat_b: f64, lon_b: f64,
    bw_horiz_deg: f64,
) -> Option<ScatterArc> {
    let gc_brg = gc_bearing_deg(lat_a, lon_a, lat_b, lon_b);
    let d_ab = gc_distance_km(lat_a, lon_a, lat_b, lon_b);

    let mut valid_offsets: Vec<f64> = Vec::new();
    let steps = (360.0 / AZ_STEP) as usize;
    for i in 0..steps {
        let az = i as f64 * AZ_STEP;
        let Some((lat_p, lon_p)) = shell_point_at_az(lat_a, lon_a, az) else {
            continue;
        };
        let el_b = elevation_angle(lat_b, lon_b, lat_p, lon_p, SHELL_KM);
        if el_b < MIN_EL_DEG {
            continue;
        }
        let offset = ((az - gc_brg + 540.0) % 360.0) - 180.0;
        valid_offsets.push(offset);
    }
    if valid_offsets.is_empty() {
        return None;
    }
    valid_offsets.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let arc_left  = *valid_offsets.first().unwrap();
    let arc_right = *valid_offsets.last().unwrap();
    let arc_half  = (arc_right - arc_left) / 2.0;

    let arc_min = (gc_brg + arc_left  + 360.0) % 360.0;
    let arc_max = (gc_brg + arc_right + 360.0) % 360.0;

    let (mid_lat, mid_lon) = destination(lat_a, lon_a, gc_brg, d_ab / 2.0);
    let mid_el = elevation_angle(lat_a, lon_a, mid_lat, mid_lon, SHELL_KM);

    let half_bw = bw_horiz_deg / 2.0;
    let (beam_a, beam_b) = if arc_half <= half_bw {
        // Arc narrower than beamwidth — one centred heading
        let centre = (gc_brg + (arc_left + arc_right) / 2.0 + 360.0) % 360.0;
        (Some(centre), None)
    } else {
        let ba = (gc_brg + arc_left  + half_bw + 360.0) % 360.0;
        let bb = (gc_brg + arc_right - half_bw + 360.0) % 360.0;
        (Some(ba), Some(bb))
    };

    Some(ScatterArc {
        gc_bearing: gc_brg,
        gc_distance_km: d_ab,
        arc_min,
        arc_max,
        arc_half_width: arc_half,
        midpoint_el: mid_el,
        beam_a,
        beam_b,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn maidenhead_4char_centre_of_square() {
        // IO82 should resolve to roughly the centre of the IO82 square:
        // longitude range −4..−2, latitude range 52..53. Centre = (52.5, −3).
        let q = Qth::from_maidenhead("IO82").unwrap();
        assert!((q.lat - 52.5).abs() < 0.01, "lat={}", q.lat);
        assert!((q.lon - (-3.0)).abs() < 0.01, "lon={}", q.lon);
    }

    #[test]
    fn maidenhead_6char_subsquare_centre() {
        let q = Qth::from_maidenhead("IO82KM").unwrap();
        // Should be inside IO82 (52..53 N, -4..-2 E) and at the centre
        // of subsquare KM. Sanity check: lat 52..53, lon -4..-2.
        assert!(q.lat > 52.0 && q.lat < 53.0, "lat={}", q.lat);
        assert!(q.lon > -4.0 && q.lon < -2.0, "lon={}", q.lon);
    }

    #[test]
    fn maidenhead_rejects_garbage() {
        assert!(Qth::from_maidenhead("").is_none());
        assert!(Qth::from_maidenhead("AB").is_none());
        assert!(Qth::from_maidenhead("12IO").is_none());      // wrong order
        // Note: trailing extra chars after a valid 6-char locator
        // are tolerated (we just take the first 4 or 6); only short
        // or malformed prefixes are rejected.
    }

    #[test]
    fn distance_io82_to_jo40_known() {
        // IO82 (Wales centre, 52.5N 3W) to JO40 (S. Germany / Black
        // Forest area, 40.5N → wait, that's 50.5N 8E).
        // Actual distance ≈ 850 km.
        let a = Qth::from_maidenhead("IO82").unwrap();
        let b = Qth::from_maidenhead("JO40").unwrap();
        let d = gc_distance_km(a.lat, a.lon, b.lat, b.lon);
        assert!(d > 750.0 && d < 1000.0, "d={}", d);
    }

    #[test]
    fn bearing_io82_to_jo40_eastish() {
        // Wales to Germany: bearing should be roughly east-southeast,
        // so somewhere between 90° and 130°.
        let a = Qth::from_maidenhead("IO82").unwrap();
        let b = Qth::from_maidenhead("JO40").unwrap();
        let brg = gc_bearing_deg(a.lat, a.lon, b.lat, b.lon);
        assert!(brg > 90.0 && brg < 130.0, "brg={}", brg);
    }

    #[test]
    fn scatter_arc_io82_to_jo40_has_finite_arc() {
        let a = Qth::from_maidenhead("IO82").unwrap();
        let b = Qth::from_maidenhead("JO40").unwrap();
        let arc = compute_scatter_arc(a.lat, a.lon, b.lat, b.lon, 50.0)
            .expect("arc should exist for short hop");
        assert!(arc.gc_distance_km > 750.0 && arc.gc_distance_km < 1000.0,
            "dist={}", arc.gc_distance_km);
        assert!(arc.gc_bearing > 90.0 && arc.gc_bearing < 130.0,
            "brg={}", arc.gc_bearing);
        // Arc should be wider than ~5° but bounded
        assert!(arc.arc_half_width > 5.0, "arc_half={}", arc.arc_half_width);
        // Midpoint elevation should be positive
        assert!(arc.midpoint_el > 0.0);
    }
}
