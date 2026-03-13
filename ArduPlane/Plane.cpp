/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "Plane.h"

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/*
  constructor for main Plane class
 */
Plane::Plane(void)
    : logger(g.log_bitmask),
    inertial_nav(ahrs),
    param_loader(var_info)
{
    // C++11 doesn't allow in-class initialisation of bitfields
    auto_state.takeoff_complete = true;

    // TIM: Added to support WP_Nav - most of this code is copied from QuadPlane::Setup()
#if AC_OAPATHPLANNER_ENABLED == ENABLED
    // Copied from
    float loop_delta_t = 1.0 / plane.scheduler.get_loop_rate_hz();
    uint16_t pwm_rate;

    // MOTORS - PosControl needs to know about motors, but if this isn't a quad plane then we just use "standard"
    //motors = new AP_MotorsMatrix(plane.scheduler.get_loop_rate_hz(), SRV_Channels::default_rate);
    AP_Param::ParamToken  token = AP_Param::ParamToken {};
    ap_var_type           type;
    AP_Param*             param = AP_Param::find_by_name("SERVO_RATE", &type, &token);
    if( param == nullptr) {
        // SERVO_RATE not found default to 50
        pwm_rate          = 50;
    } else {
        pwm_rate = ((AP_Int16 *)param)->get();
    }

    motors = new AP_MotorsMatrix(plane.scheduler.get_loop_rate_hz(), pwm_rate);
    motors_var_info = AP_MotorsMatrix::var_info;
    param_loader.load_object_from_eeprom(motors, motors_var_info);

    // AHRS_VIEW - PosControl requires an ahrs_view which seems to be a wrapper around ahrs
    // Since we are passing in ROTATION_NONE and no pitch trim, this probably does nothing except satisfy PosControl
    ahrs_view = ahrs.create_view(ROTATION_NONE);
    if (ahrs_view == nullptr) {
        AP_BoardConfig::allocation_error("ahrs_view");
    }

    // ATTITUDE CONTROL - required by AC_PosControl
    attitude_control = new AC_AttitudeControl_TS(*ahrs_view, avoidance_aparm, *motors, loop_delta_t);
    if (!attitude_control) {
        AP_BoardConfig::allocation_error("attitude_control");
    }
    param_loader.load_object_from_eeprom(attitude_control, attitude_control->var_info);

    // POSCONTROL - this will do most of the work after WPNav_OA decides on a new target, PosControl decides how to get there (I think)
    pos_control = new AC_PosControl(*ahrs_view, inertial_nav, *motors, *attitude_control, loop_delta_t);
    if (!pos_control) {
        AP_BoardConfig::allocation_error("pos_control");
    }
    param_loader.load_object_from_eeprom(pos_control, pos_control->var_info);

    // WPNAV - the OA version - incorporates all the logic we need to do dynamic/real time path planning
    wp_nav      = new AC_WPNav_OA(inertial_nav, *ahrs_view, *pos_control, *attitude_control);
    if (!wp_nav) {
        AP_BoardConfig::allocation_error("wp_nav");
    }
    AP_Param::load_object_from_eeprom(wp_nav, wp_nav->var_info);

    // Since this is for a fixed wing, we probably only have one or two forward motors. What if there are more?
    // This code was copied form QuadPlane - not sure what impact it will have, but it's requried for PosControl
    motors->init(AP_Motors::MOTOR_FRAME_SINGLE, AP_Motors::MOTOR_FRAME_TYPE_PLUS);
    motors->update_throttle_range();
    motors->set_update_rate(pwm_rate);
    attitude_control->parameter_sanity_check();
#endif
}

Plane plane;
AP_Vehicle& vehicle = plane;
