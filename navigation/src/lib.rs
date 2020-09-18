#[cfg(test)]
#[macro_use]
extern crate assert_approx_eq;

use std::f32::consts::{TAU, PI};


/// Functions use international system units unless:
/// - Latitudes and longitudes: degrees (float)
/// - Headings: degrees clockwise from North within the range [-180,180).

/// Returns the distance between two points of the surface.
/// Warning: not WGS84 compliant.
pub fn distance(from: (f32, f32), to: (f32, f32)) -> u32 {

	// Double precision is needed for intermediate calculus
	let intermediate_term = 0.5
		- (to.0 as f64 - from.0 as f64).to_radians().cos() / 2.
		+ (from.0 as f64).to_radians().cos() * (to.0 as f64).to_radians().cos() *
		(1. - (to.1 as f64 - from.1 as f64).to_radians().cos()) / 2.;

	const EARTH_RADIUS: u32 = 6371_000; /// 6371 km or 6371000 metres
	const EARTH_DIAMETER: f64 = 2. * EARTH_RADIUS as f64;

	(EARTH_DIAMETER * intermediate_term.sqrt().asin()).ceil() as u32
}

/// Returns the algebraic difference `a - b` of two angles `a` and `b` in degrees, clockwise is positive.
pub fn angle_difference_deg(a: i32, b: i32) -> i32 {
	let difference = a - b;
	let difference_abs = difference.abs() % 360;
	let smallest_angle = {
		if difference_abs > 180 {
			360 - difference_abs
		} else {
			difference_abs
		}
	};


	if (difference >= 0 && difference <= 180) || (difference <= -180 && difference >= -360) {
		smallest_angle
	} else {
		-smallest_angle
	}
}

/*
 /**
     * Returns the heading from one LatLng to another LatLng. Headings are
     * expressed in degrees clockwise from North within the range [-180,180).
     *
     * @return The heading in degrees clockwise from north.
     */
    public static double computeHeading(LatLng from, LatLng to) {
        // http://williams.best.vwh.net/avform.htm#Crs
        double fromLat = toRadians(from.latitude);
        double fromLng = toRadians(from.longitude);
        double toLat = toRadians(to.latitude);
        double toLng = toRadians(to.longitude);
        double dLng = toLng - fromLng;
        double heading = atan2(
                sin(dLng) * cos(toLat),
                cos(fromLat) * sin(toLat) - sin(fromLat) * cos(toLat) * cos(dLng));
        return wrap(toDegrees(heading), -180, 180);
    }
 */

/// Returns the initial true heading followed to travel from `a` to `b`.
/// Warning: the shortest path on the surface between two points is given by the great circle
/// crossing these two points. Therefore, heading is not constant throughout the travel.
pub fn heading(from: (f32, f32), to: (f32, f32)) -> f32 {
	let delta = to.1 as f64 - from.1 as f64;

	// Temporary use of double precision enables better accuracy
	(delta.sin() as f32 * to.0.cos())
		.atan2(from.0.cos() * to.0.sin() - from.0.sin() * to.0.cos() * delta.cos() as f32)
}

pub fn heading_deg(from: (f32, f32), to: (f32, f32)) -> f32 {
	let from = (from.0.to_radians(), from.1.to_radians());
	let to = (to.0.to_radians(), to.1.to_radians());

	heading(from, to).to_degrees()
}

/// Returns the algebraic difference `a - b` of two angles `a` and `b` in radians, clockwise is positive.
pub fn angle_difference(a: f32, b: f32) -> f32 {
	let difference = a - b;
	let difference_abs = difference.abs() % TAU;
	let smallest_angle = {
		if difference_abs > PI {
			TAU - difference_abs
		} else {
			difference_abs
		}
	};


	if (difference >= 0. && difference <= PI) || (difference <= -PI && difference >= -TAU) {
		smallest_angle
	} else {
		-smallest_angle
	}
}


#[cfg(test)]
mod tests {
	use crate::{angle_difference_deg, angle_difference, distance, heading_deg, heading};
	use std::f32::consts::{PI, FRAC_PI_2};

	#[test]
	fn angle_difference_deg_test() {
		assert_eq!(angle_difference_deg(0, 0), 0);
		assert_eq!(angle_difference_deg(10, 370), 0);
		assert_eq!(angle_difference_deg(-50, 50), -100);
		assert_eq!(angle_difference_deg(-90, 91), 179);
	}

	#[test]
	fn angle_difference_rad_test() {
		assert_approx_eq!(angle_difference(0., 0.), 0.);
		assert_approx_eq!(angle_difference(10f32.to_radians(), 370f32.to_radians()), 0.);
		assert_approx_eq!(angle_difference(-50f32.to_radians(), 50f32.to_radians()), -100f32.to_radians());
		assert_approx_eq!(angle_difference(-90f32.to_radians(), 91f32.to_radians()), 179f32.to_radians());
	}

	#[test]
	fn distance_test() {
		// Distance between LFRN and LFPO
		assert_eq!(distance((48.0708536, -1.7329389), (48.7322773, 2.3540089)), 310_503);
		// Distance between Telecom Paris and the Eiffel tower
		assert_eq!(distance((48.713202, 2.200641), (48.858404, 2.294534)), 17_550);
	}

	#[test]
	fn heading_deg_test() {
		assert_eq!(heading_deg((48.0708536, 0.), (48.0708536, 0.0001)), 90.);
		assert_eq!(heading_deg((48.0708536, 0.0001), (48.0708536, 0.)), -90.);
		assert_eq!(heading_deg((48.0708536, 0.), (48.1708536, 0.)), 0.);
		// Heading from LFRN (N48°4.40' W1°44.02') to LFRD (N48°35.50' W2°4.85')
		let lfrn = (48. + 4.40 / 60., -(1. + 44.02 / 60.));
		let lfrd = (48. + 35.50 / 60., -(2. + 4.85 / 60.));
		assert_approx_eq!(heading_deg(lfrn, lfrd), -24., 1.);
		assert_approx_eq!(heading_deg(lfrd, lfrn), -24. + 180., 1.);
		// LFPO N48°43.47' E2°22.84'
		let lfpo = (48. + 43.47 / 60., 2. + 22.84 / 60.);
		assert_approx_eq!(heading_deg(lfrn, lfpo), 75., 1.)
	}

	#[test]
	fn heading_test() {
		assert_eq!(heading((0.0708536, 0.), (0.0708536, 0.0001)), FRAC_PI_2);
		assert_eq!(heading((0.0708536, 0.0001), (0.0708536, 0.)), -FRAC_PI_2);
		assert_eq!(heading((0.0708536, 0.), (0.1708536, 0.)), 0.);
		let lfrn = ((48f32 + 4.40 / 60.).to_radians(), (-(1f32 + 44.02 / 60.)).to_radians());
		let lfrd = ((48f32 + 35.50 / 60.).to_radians(), (-(2f32 + 4.85 / 60.)).to_radians());
		assert_approx_eq!(heading(lfrn, lfrd), -24f32.to_radians(), 1e-2);
		assert_approx_eq!(heading(lfrd, lfrn), -24f32.to_radians() + PI, 1e-2);
	}
}
