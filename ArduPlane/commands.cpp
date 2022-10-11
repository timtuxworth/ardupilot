/*
 *  logic for dealing with the current command in the mission and home location
 */

#include "Plane.h"

#if AC_OAPATHPLANNER_ENABLED == ENABLED
/* 
 * If object avoidance is active, pass the new waypoint to the back end object avoidance thread
 * TIM: This code/logic was copied from AC_WPNav_OA.cpp bool AC_WPNav_OA::update_wpnav()
 */
bool Plane::update_oanav(Location &oa_origin, Location &oa_destination)
{
    /* TIM: Delete this - just for WIP
    AP_OAPathPlanner::OA_RetState AP_OAPathPlanner::mission_avoidance(const Location &current_loc,
                                         const Location &origin,
                                         const Location &destination,
                                         Location &result_origin,
                                         Location &result_destination,
                                         OAPathPlannerUsed &path_planner_used)
    */
    // run path planning around obstacles
    AP_OAPathPlanner *oa_ptr = AP_OAPathPlanner::get_singleton();
    
    Location starting_loc;
    if(oa_ptr == nullptr || !AP::ahrs().get_location(starting_loc))
        return false;

    // backup _origin and _destination when not doing oa
    // TIM: I really don't understand what this is trying to do
    /*
    if (_oa_state == AP_OAPathPlanner::OA_NOT_REQUIRED) {
        _origin_oabak = _origin;
        _destination_oabak = _destination;
        _terrain_alt_oabak = _terrain_alt;
    }
    */

    // TIM: ORiginal code from AC_WPNav_OA
    /*
    // convert origin and destination to Locations and pass into oa 
    const Location origin_loc(_origin_oabak, _terrain_alt_oabak ? Location::AltFrame::ABOVE_TERRAIN : Location::AltFrame::ABOVE_ORIGIN);
    const Location destination_loc(_destination_oabak, _terrain_alt_oabak ? Location::AltFrame::ABOVE_TERRAIN : Location::AltFrame::ABOVE_ORIGIN);
    const AP_OAPathPlanner::OA_RetState oa_retstate = oa_ptr->mission_avoidance(starting_loc, origin_loc, destination_loc, oa_origin_new, oa_destination_new, path_planner_used);
    */

    Location oa_origin_new, oa_destination_new;
    AP_OAPathPlanner::OAPathPlannerUsed path_planner_used = AP_OAPathPlanner::OAPathPlannerUsed::None;
    const AP_OAPathPlanner::OA_RetState oa_retstate = oa_ptr->mission_avoidance(starting_loc, oa_origin, oa_destination, oa_origin_new, oa_destination_new, path_planner_used);
    
    switch (oa_retstate) {

    case AP_OAPathPlanner::OA_NOT_REQUIRED:
        if (_oa_state != oa_retstate) {
            // object avoidance has become inactive so reset target to original destination
            // TIM: was set_wp_destination(_destination_oabak, _terrain_alt_oabak);
            set_next_WP(oa_destination);
            _oa_state = oa_retstate;
        }
        break;

    case AP_OAPathPlanner::OA_PROCESSING:
    case AP_OAPathPlanner::OA_ERROR:
        _oa_state = oa_retstate;
        /* TIM: PLane can't stop. Figure this out later. Not sure why we would stop if still OA_PROCESSING anyway
        // during processing or in case of error stop the vehicle
        // by setting the oa_destination to a stopping point
        if ((_oa_state != AP_OAPathPlanner::OA_PROCESSING) && (_oa_state != AP_OAPathPlanner::OA_ERROR)) {
            // calculate stopping point
            Vector3f stopping_point;
            get_wp_stopping_point(stopping_point);
            _oa_destination = Location(stopping_point, Location::AltFrame::ABOVE_ORIGIN);
            if (set_wp_destination(stopping_point, false)) {
                _oa_state = oa_retstate;
            }
           
        }
        */
        break;

    case AP_OAPathPlanner::OA_SUCCESS:
        // The path planning completed processing. It will have set a new destination in oa_destination_new
        // handling of returned destination depends upon path planner used
        switch (path_planner_used) {

        case AP_OAPathPlanner::OAPathPlannerUsed::None: {
            // this should never happen.  this means the path planner has returned success but has failed to set the path planner used
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            return false;
        }

        case AP_OAPathPlanner::OAPathPlannerUsed::Dijkstras: {
            // Dijkstra's.  Action is only needed if path planner has just became active or the target destination's lat or lon has changed
            if ((_oa_state != AP_OAPathPlanner::OA_SUCCESS) || !oa_destination_new.same_latlon_as(_oa_destination)) {
                Location origin_oabak_loc(_origin_oabak, _terrain_alt_oabak ? Location::AltFrame::ABOVE_TERRAIN : Location::AltFrame::ABOVE_ORIGIN);
                Location destination_oabak_loc(_destination_oabak, _terrain_alt_oabak ? Location::AltFrame::ABOVE_TERRAIN : Location::AltFrame::ABOVE_ORIGIN);
                oa_destination_new.linearly_interpolate_alt(origin_oabak_loc, destination_oabak_loc);
                // TIM: was if (!set_wp_destination_loc(oa_destination_new)) {
                set_next_WP(oa_destination_new);
                // if new target set successfully, update oa state and destination
                _oa_state = oa_retstate;
                _oa_destination = oa_destination_new;
            }
            break;
        }

        case AP_OAPathPlanner::OAPathPlannerUsed::BendyRulerHorizontal: {
            set_next_WP(oa_destination_new);
            _oa_state = oa_retstate;
            _oa_destination = oa_destination_new;

            // altitude target interpolated from starting_loc's distance along the original path
            Location target_alt_loc = starting_loc;
            target_alt_loc.linearly_interpolate_alt(oa_origin, oa_destination);

            /* TIM: How does this work for plane?
            // correct target_alt_loc's alt-above-ekf-origin if using terrain altitudes
            // positive terr_offset means terrain below vehicle is above ekf origin's altitude
            float terr_offset = 0;
            if (_terrain_alt_oabak && !get_terrain_offset(terr_offset)) {
                // trigger terrain failsafe
                return false;
            }

            // calculate final destination as an offset from EKF origin in NEU
            Vector2f dest_NE;
            if (!_oa_destination.get_vector_xy_from_origin_NE(dest_NE)) {
                // this should never happen because we can only get here if we have an EKF origin
                INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                return false;
            }
            Vector3p dest_NEU{dest_NE.x, dest_NE.y, (float)target_alt_loc.alt};

            // pass the desired position directly to the position controller
            _pos_control.input_pos_xyz(dest_NEU, terr_offset, 1000.0);

            // update horizontal position controller (vertical is updated in vehicle code)
            _pos_control.update_xy_controller();
            */

            // return success without calling parent AC_WPNav
            break;
        }
        
        case AP_OAPathPlanner::OAPathPlannerUsed::BendyRulerVertical: {
            set_next_WP(oa_destination_new);
            _oa_state = oa_retstate;
            _oa_destination = oa_destination_new;

            /* TIM: NEed to figure out how to do this with plane ToDo
            // calculate final destination as an offset from EKF origin in NEU
            Vector3f dest_NEU;
            if (!_oa_destination.get_vector_from_origin_NEU(dest_NEU)) {
                // this should never happen because we can only get here if we have an EKF origin
                INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                return false;
            }

            // pass the desired position directly to the position controller as an offset from EKF origin in NEU
            Vector3p dest_NEU_p{dest_NEU.x, dest_NEU.y, dest_NEU.z};
            _pos_control.input_pos_xyz(dest_NEU_p, 0, 1000.0);

            // update horizontal position controller (vertical is updated in vehicle code)
            _pos_control.update_xy_controller();

            // return success without calling parent AC_WPNav
            */
    
            break;
        }
        }
    }
    return true;
}
#endif

/*
 *  set_next_WP - sets the target location the vehicle should fly to
 */
void Plane::set_next_WP(const struct Location &loc)
{
    if (auto_state.next_wp_crosstrack) {
        // copy the current WP into the OldWP slot
        prev_WP_loc = next_WP_loc;
        auto_state.crosstrack = true;
    } else {
        // we should not try to cross-track for this waypoint
        prev_WP_loc = current_loc;
        // use cross-track for the next waypoint
        auto_state.next_wp_crosstrack = true;
        auto_state.crosstrack = false;
    }

    // Load the next_WP slot
    // ---------------------
    next_WP_loc = loc;

    // if lat and lon is zero, then use current lat/lon
    // this allows a mission to contain a "loiter on the spot"
    // command
    if (next_WP_loc.lat == 0 && next_WP_loc.lng == 0) {
        next_WP_loc.lat = current_loc.lat;
        next_WP_loc.lng = current_loc.lng;
        // additionally treat zero altitude as current altitude
        if (next_WP_loc.alt == 0) {
            next_WP_loc.alt = current_loc.alt;
            next_WP_loc.relative_alt = false;
            next_WP_loc.terrain_alt = false;
        }
    }

    // convert relative alt to absolute alt
    if (next_WP_loc.relative_alt) {
        next_WP_loc.relative_alt = false;
        next_WP_loc.alt += home.alt;
    }

    // are we already past the waypoint? This happens when we jump
    // waypoints, and it can cause us to skip a waypoint. If we are
    // past the waypoint when we start on a leg, then use the current
    // location as the previous waypoint, to prevent immediately
    // considering the waypoint complete
    if (current_loc.past_interval_finish_line(prev_WP_loc, next_WP_loc)) {
        prev_WP_loc = current_loc;
    }

    // zero out our loiter vals to watch for missed waypoints
    loiter_angle_reset();

    setup_glide_slope();
    setup_turn_angle();

#if AC_OAPATHPLANNER_ENABLED == ENABLED
    // TIM: not sure if I understand origin - here I am using the Previous WP as the "origin", because some code in AC_WPNav does this. Not sure if it's right
    Plane::update_oanav(prev_WP_loc, next_WP_loc);
#endif
}

void Plane::set_guided_WP(const Location &loc)
{
    if (aparm.loiter_radius < 0 || loc.loiter_ccw) {
        loiter.direction = -1;
    } else {
        loiter.direction = 1;
    }

    // copy the current location into the OldWP slot
    // ---------------------------------------
    prev_WP_loc = current_loc;

    // Load the next_WP slot
    // ---------------------
    next_WP_loc = loc;

    // used to control FBW and limit the rate of climb
    // -----------------------------------------------
    set_target_altitude_current();

    setup_glide_slope();
    setup_turn_angle();

    // disable crosstrack, head directly to the point
    auto_state.crosstrack = false;

    // reset loiter start time.
    loiter.start_time_ms = 0;

    // start in non-VTOL mode
    auto_state.vtol_loiter = false;
    
    loiter_angle_reset();

#if HAL_QUADPLANE_ENABLED
    // cancel pending takeoff
    quadplane.guided_takeoff = false;
#endif
#if AC_OAPATHPLANNER_ENABLED == ENABLED
    update_oanav(prev_WP_loc, next_WP_loc);
#endif
}

/*
  update home location from GPS
  this is called as long as we have 3D lock and the arming switch is
  not pushed
*/
void Plane::update_home()
{
    if (hal.util->was_watchdog_armed()) {
        return;
    }
    if ((g2.home_reset_threshold == -1) ||
        ((g2.home_reset_threshold > 0) &&
         (fabsf(barometer.get_altitude()) > g2.home_reset_threshold))) {
        // don't auto-update if we have changed barometer altitude
        // significantly. This allows us to cope with slow baro drift
        // but not re-do home and the baro if we have changed height
        // significantly
        return;
    }
    if (ahrs.home_is_set() && !ahrs.home_is_locked()) {
        Location loc;
        if(ahrs.get_location(loc) && gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
            // we take the altitude directly from the GPS as we are
            // about to reset the baro calibration. We can't use AHRS
            // altitude or we can end up perpetuating a bias in
            // altitude, as AHRS alt depends on home alt, which means
            // we would have a circular dependency
            loc.alt = gps.location().alt;
            if (!AP::ahrs().set_home(loc)) {
                // silently fail
            }
        }
    }
    barometer.update_calibration();
    ahrs.resetHeightDatum();
}

bool Plane::set_home_persistently(const Location &loc)
{
    if (hal.util->was_watchdog_armed()) {
        return false;
    }
    if (!AP::ahrs().set_home(loc)) {
        return false;
    }

    // Save Home to EEPROM
    mission.write_home_to_storage();

    return true;
}
