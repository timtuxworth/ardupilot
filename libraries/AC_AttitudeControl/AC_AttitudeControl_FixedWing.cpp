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

   This class inherits from AC_AttitudeControl_Multi and provides functionality
   specific to fixed wing planes.

   This is a wrapper around the non-OO attitude controller in ArduPlane to allow FixedWing to use WP_Nav
   Ideally the ArduPlane/Attidude.cpp code would be moved here eventually.

 */


#include "AC_AttitudeControl_FixedWing.h"

//  AC_AttitudeControl_FixedWing(AP_AHRS_View &ahrs, const AP_Vehicle::FixedWing &aparm, float dt);
//  A Fixed Wing does not have "motors" in the sense of multirotors, so the motors is passes as null to AC_Attitude_Control

AC_AttitudeControl_FixedWing::AC_AttitudeControl_FixedWing(AP_AHRS_View &ahrs, const AP_Vehicle::FixedWing &aparm, float dt) :
    AC_AttitudeControl(ahrs, aparm, nullptr, dt)
    //,
    //_pid_rate_roll(AC_ATC_MULTI_RATE_RP_P, AC_ATC_MULTI_RATE_RP_I, AC_ATC_MULTI_RATE_RP_D, 0.0f, AC_ATC_MULTI_RATE_RP_IMAX, AC_ATC_MULTI_RATE_RP_FILT_HZ, 0.0f, AC_ATC_MULTI_RATE_RP_FILT_HZ, dt),
    //_pid_rate_pitch(AC_ATC_MULTI_RATE_RP_P, AC_ATC_MULTI_RATE_RP_I, AC_ATC_MULTI_RATE_RP_D, 0.0f, AC_ATC_MULTI_RATE_RP_IMAX, AC_ATC_MULTI_RATE_RP_FILT_HZ, 0.0f, AC_ATC_MULTI_RATE_RP_FILT_HZ, dt),
    //_pid_rate_yaw(AC_ATC_MULTI_RATE_YAW_P, AC_ATC_MULTI_RATE_YAW_I, AC_ATC_MULTI_RATE_YAW_D, 0.0f, AC_ATC_MULTI_RATE_YAW_IMAX, AC_ATC_MULTI_RATE_RP_FILT_HZ, AC_ATC_MULTI_RATE_YAW_FILT_HZ, 0.0f, dt)
{
    AP_Param::setup_object_defaults(this, var_info);
}

/*
void AC_AttitudeControl_FixedWing::relax_attitude_controllers(bool exclude_pitch)
{
    // If exclude_pitch: relax roll and yaw rate controller outputs only,
    // leaving pitch controller active to let TVBS motors tilt up while in throttle_wait
    if (exclude_pitch) {
        // Get the current attitude quaternion
        Quaternion current_attitude;
        _ahrs.get_quat_body_to_ned(current_attitude);

        Vector3f current_eulers;
        current_attitude.to_euler(current_eulers.x, current_eulers.y, current_eulers.z);

        // set target attitude to zero pitch with (approximate) current roll and yaw
        // by rotating the current_attitude quaternion by the error in desired pitch
        Quaternion pitch_rotation;
        pitch_rotation.from_axis_angle(Vector3f(0, -1, 0), current_eulers.y);
        _attitude_target = current_attitude * pitch_rotation;
        _attitude_target.normalize();
        _attitude_target.to_euler(_euler_angle_target.x, _euler_angle_target.y, _euler_angle_target.z);
        _attitude_ang_error = current_attitude.inverse() * _attitude_target;

        // Initialize the roll and yaw angular rate variables to the current rate
        _ang_vel_target = _ahrs.get_gyro();
        ang_vel_to_euler_rate(_euler_angle_target, _ang_vel_target, _euler_rate_target);
        _ang_vel_body.x = _ahrs.get_gyro().x;
        _ang_vel_body.z = _ahrs.get_gyro().z;

        // Reset the roll and yaw I terms
        get_rate_roll_pid().reset_I();
        get_rate_yaw_pid().reset_I();
    } else {
        // relax all attitude controllers
        AC_AttitudeControl::relax_attitude_controllers();
    }
}

// Command euler yaw rate and pitch angle with roll angle specified in body frame
// (used only by tailsitter quadplanes)
// If plane_controls is true, swap the effects of roll and yaw as euler pitch approaches 90 degrees
void AC_AttitudeControl_FixedWing::input_euler_rate_yaw_euler_angle_pitch_bf_roll(bool plane_controls, float body_roll_cd, float euler_pitch_cd, float euler_yaw_rate_cds)
{
    // Convert from centidegrees on public interface to radians
    float euler_yaw_rate = radians(euler_yaw_rate_cds*0.01f);
    float euler_pitch    = radians(constrain_float(euler_pitch_cd * 0.01f, -90.0f, 90.0f));
    float body_roll      = radians(-body_roll_cd * 0.01f);

    const float cpitch = cosf(euler_pitch);
    const float spitch = fabsf(sinf(euler_pitch));

    // Compute attitude error
    Quaternion attitude_body;
    Quaternion error_quat;
    _ahrs.get_quat_body_to_ned(attitude_body);
    error_quat = attitude_body.inverse() * _attitude_target;
    Vector3f att_error;
    error_quat.to_axis_angle(att_error);

    // update heading
    float yaw_rate = euler_yaw_rate;
    if (plane_controls) {
        yaw_rate = (euler_yaw_rate * spitch) + (body_roll * cpitch);
    }
    // limit yaw error
    float yaw_error = fabsf(att_error.z);
    float error_ratio = yaw_error / M_PI_2;
    if (error_ratio > 1) {
        yaw_rate /= (error_ratio * error_ratio);
    }
    _euler_angle_target.z = wrap_PI(_euler_angle_target.z + yaw_rate * _dt);

    // init attitude target to desired euler yaw and pitch with zero roll
    _attitude_target.from_euler(0, euler_pitch, _euler_angle_target.z);

    // apply body-frame yaw/roll (this is roll/yaw for a tailsitter in forward flight)
    // rotate body_roll axis by |sin(pitch angle)|
    Quaternion bf_roll_Q;
    bf_roll_Q.from_axis_angle(Vector3f(0, 0, spitch * body_roll));

    // rotate body_yaw axis by cos(pitch angle)
    Quaternion bf_yaw_Q;
    if (plane_controls) {
        bf_yaw_Q.from_axis_angle(Vector3f(cpitch, 0, 0), euler_yaw_rate);
    } else {
        bf_yaw_Q.from_axis_angle(Vector3f(-cpitch * body_roll, 0, 0));
    }
    _attitude_target = _attitude_target * bf_roll_Q * bf_yaw_Q;

    // _euler_angle_target roll and pitch: Note: roll/yaw will be indeterminate when pitch is near +/-90
    // These should be used only for logging target eulers, with the caveat noted above.
    // Also note that _attitude_target.from_euler() should only be used in special circumstances
    // such as when attitude is specified directly in terms of Euler angles.
    //    _euler_angle_target.x = _attitude_target.get_euler_roll();
    //    _euler_angle_target.y = euler_pitch;

    // Set rate feedforward requests to zero
    _euler_rate_target.zero();
    _ang_vel_target.zero();

    // Compute attitude error
    error_quat = attitude_body.inverse() * _attitude_target;
    error_quat.to_axis_angle(att_error);

    // Compute the angular velocity target from the attitude error
    _ang_vel_body = update_ang_vel_target_from_att_error(att_error);
}
*/

// Required by AC_WPNav and AC_WPNAv_OA
float AC_AttitudeControl_FixedWing::get_accel_max()
{

}

float AC_AttitudeControl_FixedWing::get_decel_max()
{

}

float AC_AttitudeControl_FixedWing::get_turn_lat_accel_max()
{

}

// Forward speed for a fixed wing is easy - it's whatever AHRS thinks it is! 
// Will this use synthetic airspeed if there is no sensor?
void AC_AttitudeControl_FixedWing::get_forward_speed(float &speed)
{
    (void)_ahrs.airspeed_estimate(aspeed);
}
float AC_AttitudeControl_FixedWing::get_desired_speed_accel_limited(float desired_speed, float dt)
{

}
// A fixed wing can't stop (hmm - what about deploying a parachute?) - so for simplicity we assume stopping distance is infinite
float AC_AttitudeControl_FixedWing::get_stopping_distance(float speed)
{
    return FLT_MAX;
}