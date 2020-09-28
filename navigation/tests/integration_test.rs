#[cfg(test)]
#[macro_use]
extern crate assert_approx_eq;

use navigation;

#[test]
pub fn heading_distance_test() {
    // Distance and heading from Rennes to Dinard (336°, 34nm)
    let lfrn = (48f32 + 4.40 / 60., -(1f32 + 44.02 / 60.));
    let lfrd = (48f32 + 35.50 / 60., -(2f32 + 4.85 / 60.));
    let heading = navigation::heading_deg(lfrn, lfrd);
    let distance = navigation::distance(lfrn, lfrd);

    assert_approx_eq!(heading, 336. - 360., 0.5);

    const NM: u32 = 1852;
    assert_eq!(distance / NM, 34);

    // Distance and heading from New York to Paris
    // 40° 38′ 26″ nord, 73° 46′ 44″ ouest

    let kjfk = (
        40. + 38. / 60. + 26. / 3600.,
        -(73. + 46. / 60. + 44. / 3600.),
    );
    // 49° 00′ 36″ nord, 2° 32′ 55″ est
    let lfpg = (40. + 36. / 3600., 2. + 32. / 60. + 55. / 3600.);
    let heading = navigation::heading_deg(kjfk, lfpg);
    let distance = navigation::distance(kjfk, lfpg);

    assert_approx_eq!(heading, 66., 2.5);
    assert_approx_eq!(distance as f32 / NM as f32, 3158., 220.); // We cannot expect super high
                                                                 // accuracy because of the non-compliance with WGS84. Some website such as
                                                                 // `https://flightplandatabase.com/` estimate the distance to be 3151.3 nm, which is close to
                                                                 // the value given by `distance` function.

    // Distance and heading from Telecom Paris to Polytechnique
    let telecom_paris = (48.712914, 2.200367);
    let polytechnique = (48.713375, 2.210051);
    let heading = navigation::heading_deg(telecom_paris, polytechnique);
    let distance = navigation::distance(telecom_paris, polytechnique);

    assert_approx_eq!(heading, 86., 0.5);
    assert_eq!(distance, 713);
}
