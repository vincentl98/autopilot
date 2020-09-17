#[cfg(test)]
#[macro_use]
extern crate assert_approx_eq;

use std::f32::consts::{TAU, PI};
use std::marker::PhantomData;

/// Functions use international system units unless:
/// - Latitudes and longitudes: degrees (float)
/// - Headings: degrees clockwise from North within the range [-180,180).

/// Returns the distance between two points of the surface.
/// Warning: not WGS84 compliant.
pub fn distance((from_lat, from_lng): (f32, f32), (to_lat, to_lng): (f32, f32)) -> u32 {
	let intermediate_term = 0.5
		- (to_lat - from_lat).to_radians().cos() / 2.
		+ from_lat.to_radians().cos() * to_lat.to_radians().cos() *
		(1. - (to_lng - from_lng).to_radians().cos()) / 2.;

	const EARTH_RADIUS: u32 = 6371_000; /// 6371 km or 6371000 metres
	const EARTH_DIAMETER: f32 = 2. * EARTH_RADIUS as f32;

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
	let delta = to.1 - from.1;
	(delta.sin() * to.0.cos())
		.atan2(from.0.cos() * to.0.sin() - from.0.sin() * to.0.cos() * delta.cos())
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
	use crate::{angle_difference_deg, angle_difference, distance};

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
		assert_eq!(distance((48.0708536, -1.7329389), (48.7322773, 2.3540089)), 310_505);
		// Distance between Telecom Paris and the Eiffel tower
		assert_eq!(distance((48.713202, 2.200641), (48.858404, 2.294534)), 17_596);
	}

	#[test]
	fn heading_deg_test() {

	}
}
