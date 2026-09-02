#include "Copter.h"

// read_inertia - read inertia in from accelerometers
void Copter::read_inertia()
{
    // inertial altitude estimates. Use barometer climb rate during high vibrations
    pos_control->update_estimates(vibration_check.high_vibes);
#if MODE_FOLLOW_ENABLED
    g2.follow.update_estimates();
#if HAL_MOUNT_ENABLED
    // let the mount use AP_Follow's kinematic estimate for SYSID_TARGET
    // tracking if it's following the same vehicle; pushed here (rather than
    // AP_Mount reaching into AP_Follow directly) to avoid a library-to-
    // library dependency - see PR #34237 review discussion
    if (g2.follow.enabled()) {
        const uint8_t follow_sysid = (uint8_t)g2.follow.get_target_sysid();
        camera_mount.set_target_sysid_kinematic_active(follow_sysid);
        Vector3p pos_ned_m;
        Vector3f vel_ned_ms, accel_ned_mss;
        if (g2.follow.get_target_pos_vel_accel_NED_m(pos_ned_m, vel_ned_ms, accel_ned_mss)) {
            Location loc;
            if (AP::ahrs().get_location_from_origin_offset_NED(loc, pos_ned_m)) {
                camera_mount.set_target_sysid_kinematic_estimate(follow_sysid, loc);
            }
        }
    }
#endif  // HAL_MOUNT_ENABLED
#endif  // MODE_FOLLOW_ENABLED

    // pull position from ahrs
    Location loc;
    // AHRS provides a best-guess in case of failure
    UNUSED_RESULT(ahrs.get_location(loc));
    current_loc.lat = loc.lat;
    current_loc.lng = loc.lng;

    // exit immediately if we do not have an altitude estimate
    float pos_d_m;
    if (!AP::ahrs().get_relative_position_D_origin_float(pos_d_m)) {
        return;
    }

    // current_loc.alt is alt-above-home, converted from AHRS's alt-above-ekf-origin
    const float alt_above_origin_m = -pos_d_m;
    current_loc.set_alt_m(alt_above_origin_m, Location::AltFrame::ABOVE_ORIGIN);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (!current_loc.initialised()) {
        // this can happen if we don't have lat/lng and our altitude
        // is *very* close the origin i.e.  lat=0, lng=0 and
        // int32_t(abs(pos_d_m)*100)==0
        return;
    }
#endif  // CONFIG_HAL_BOARD == HAL_BOARD_SITL

    if (!ahrs.home_is_set() || !current_loc.change_alt_frame(Location::AltFrame::ABOVE_HOME)) {
        // if home has not been set yet we treat alt-above-origin as alt-above-home
        current_loc.set_alt_m(alt_above_origin_m, Location::AltFrame::ABOVE_HOME);
    }
}
