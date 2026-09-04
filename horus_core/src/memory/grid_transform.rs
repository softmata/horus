//! Shared map-origin transforms for the grid-shaped message types.
//!
//! `OccupancyGrid` and `CostMap` both carry an origin `(x, y, theta)` and both
//! need the same pair of transforms between world coordinates and the map
//! frame. Keeping one copy here is not tidiness: the two directions must stay
//! exact inverses of each other, and `world_to_grid`/`grid_to_world` on either
//! type round-tripping is the property the callers rely on. Four hand-written
//! copies of the same trigonometry is four places for that property to rot.

/// Transform world coordinates into the map frame.
///
/// Translates by the origin, then UNDOES the map's rotation, leaving a
/// displacement in metres along the grid's own axes. Divide by the resolution
/// to get cell coordinates.
///
/// `theta == 0.0` short-circuits to plain translation, so unrotated maps --
/// the overwhelmingly common case -- get bit-for-bit the arithmetic they had
/// before rotation was honoured, and pay nothing for the trig.
///
/// Returns `None` if the result is not finite. `origin_theta` reaches this
/// code from a Pod descriptor that travels through shared memory, so any f64
/// bit pattern is reachable, including NaN and the infinities; `sin_cos()` on
/// those yields NaN, and `(NaN.floor() as i32)` saturates to 0 rather than
/// trapping. Without this check a poisoned origin turns every lookup into a
/// confident, in-range, wrong cell (0, 0). Refusing is the same answer the
/// caller already handles for an out-of-map query.
#[inline]
pub(crate) fn world_to_map(
    origin_x: f64,
    origin_y: f64,
    theta: f64,
    x: f64,
    y: f64,
) -> Option<(f64, f64)> {
    let (dx, dy) = (x - origin_x, y - origin_y);
    let (mx, my) = if theta == 0.0 {
        (dx, dy)
    } else {
        let (sin_t, cos_t) = theta.sin_cos();
        (dx * cos_t + dy * sin_t, -dx * sin_t + dy * cos_t)
    };
    if mx.is_finite() && my.is_finite() {
        Some((mx, my))
    } else {
        None
    }
}

/// Transform map-frame coordinates (metres along the grid's own axes) into
/// world coordinates.
///
/// Exact inverse of [`world_to_map`]: rotate INTO the world frame, then
/// translate by the origin. Same `theta == 0.0` short-circuit, same
/// non-finite refusal, for the same reasons.
#[inline]
pub(crate) fn map_to_world(
    origin_x: f64,
    origin_y: f64,
    theta: f64,
    mx: f64,
    my: f64,
) -> Option<(f64, f64)> {
    let (rx, ry) = if theta == 0.0 {
        (mx, my)
    } else {
        let (sin_t, cos_t) = theta.sin_cos();
        (mx * cos_t - my * sin_t, mx * sin_t + my * cos_t)
    };
    let (x, y) = (origin_x + rx, origin_y + ry);
    if x.is_finite() && y.is_finite() {
        Some((x, y))
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    /// theta == 0 must be the plain translation, to the bit -- every existing
    /// caller has an unrotated map and must not drift.
    #[test]
    fn zero_theta_is_exact_translation() {
        assert_eq!(world_to_map(1.5, -2.25, 0.0, 4.0, 6.0), Some((2.5, 8.25)));
        assert_eq!(map_to_world(1.5, -2.25, 0.0, 2.5, 8.25), Some((4.0, 6.0)));
    }

    /// The two directions are inverses, which is what makes
    /// `grid -> world -> grid` return the cell it started from.
    #[test]
    fn directions_are_inverses() {
        let (ox, oy, theta) = (1.0, -2.0, 0.7);
        for &(mx, my) in &[(0.0, 0.0), (0.35, 1.2), (-4.0, 9.5)] {
            let (wx, wy) = map_to_world(ox, oy, theta, mx, my).expect("finite");
            let (bx, by) = world_to_map(ox, oy, theta, wx, wy).expect("finite");
            assert!(
                (bx - mx).abs() < 1e-9 && (by - my).abs() < 1e-9,
                "({mx}, {my}) -> ({wx}, {wy}) -> ({bx}, {by})"
            );
        }
    }

    /// A quarter turn sends map +x onto world +y. Guards against the pair
    /// being consistently wrong (e.g. both rotating the wrong way), which the
    /// inverse test alone would not catch.
    #[test]
    fn quarter_turn_sends_map_x_to_world_y() {
        let (x, y) = map_to_world(0.0, 0.0, FRAC_PI_2, 3.0, 0.0).expect("finite");
        assert!(
            (x - 0.0).abs() < 1e-9 && (y - 3.0).abs() < 1e-9,
            "({x}, {y})"
        );
    }

    /// A non-finite origin_theta must refuse, not answer. `sin_cos()` on NaN
    /// is NaN, and the caller's `as i32` would saturate that to a plausible
    /// in-range cell (0, 0).
    #[test]
    fn non_finite_theta_is_refused() {
        for theta in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert_eq!(
                world_to_map(1.0, 2.0, theta, 3.0, 4.0),
                None,
                "theta {theta} must not produce coordinates"
            );
            assert_eq!(
                map_to_world(1.0, 2.0, theta, 3.0, 4.0),
                None,
                "theta {theta} must not produce coordinates"
            );
        }
    }

    /// The same refusal covers a non-finite origin or query point, which reach
    /// this code from the same untrusted descriptor bytes.
    #[test]
    fn non_finite_origin_or_point_is_refused() {
        assert_eq!(world_to_map(f64::NAN, 0.0, 0.0, 1.0, 1.0), None);
        assert_eq!(world_to_map(0.0, 0.0, 0.0, f64::NAN, 1.0), None);
        assert_eq!(map_to_world(0.0, f64::NAN, 0.0, 1.0, 1.0), None);
        // Infinity minus infinity is NaN, so an infinite origin is refused
        // even when the query point is infinite too.
        assert_eq!(
            world_to_map(f64::INFINITY, 0.0, 0.0, f64::INFINITY, 1.0),
            None
        );
    }
}
