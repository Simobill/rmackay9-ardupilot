#include "Copter.h"

/*
 * mode_chase.cpp - chase another mavlink-enabled vehicle by system id
 *
 * TODO: set ROI yaw mode / point camera at target
 * TODO: stick control to move around on sphere
 * TODO: stick control to change sphere diameter
 * TODO: "channel 7 option" to lock onto "pointed at" target
 * TODO: do better in terms of loitering around the moving point; may need a PID?  Maybe use loiter controller somehow?
 * TODO: extrapolate target vehicle position using its velocity and acceleration
 * TODO: ensure AC_AVOID_ENABLED is true because we rely on it velocity limiting functions
 */

#if 0
#define Debug(fmt, args ...)  do {::fprintf(stderr, "%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)
#else
#define Debug(fmt, args ...)
#endif

// initialise avoid_adsb controller
bool Copter::ModeChase::init(const bool ignore_checks)
{
    // re-use guided mode
    return Copter::ModeGuided::init(ignore_checks);
}

bool Copter::ModeChase::set_velocity(const Vector3f& velocity_neu)
{
    // check flight mode
    if (_copter.flightmode != &_copter.mode_chase) {
        return false;
    }

    return true;
}

void Copter::ModeChase::run()
{
    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock()) {
        zero_throttle_and_relax_ac();
        return;
    }

    // re-use guided mode's velocity controller
    // Note: this is safe from interference from GCSs and companion computer's whose guided mode
    //       position and velocity requests will be ignored while the vehicle is not in guided mode

    Vector3f desired_velocity;

    Vector3f dist_vec_to_target;
    if (g2.follow.get_distance_to_target_ned(dist_vec_to_target)) {
        // debug
        Debug("dist to vec: %f %f %f", (double)dist_vec_to_target.x, (double)dist_vec_to_target.y, (double)dist_vec_to_target.z);

        // convert dist_vec_to_target to cm in NEU
        Vector3f dist_vec_to_target_neu(dist_vec_to_target.x * 100.0f, dist_vec_to_target.y * 100.0f, -dist_vec_to_target.z * 100.0f);

        // calculate desired velocity vector in cm/s in NEU
        desired_velocity.x = dist_vec_to_target_neu.x * pos_control->get_pos_xy_p().kP();
        desired_velocity.y = dist_vec_to_target_neu.y * pos_control->get_pos_xy_p().kP();
        desired_velocity.z = dist_vec_to_target_neu.z * pos_control->get_pos_z_p().kP();

        // scale desired velocity to stay within horizontal speed limit
        float desired_speed_xy = safe_sqrt(sq(desired_velocity.x) + sq(desired_velocity.y));
        if (!is_zero(desired_speed_xy) && (desired_speed_xy > pos_control->get_speed_xy())) {
            const float scalar_xy = pos_control->get_speed_xy() / desired_speed_xy;
            desired_velocity.x *= scalar_xy;
            desired_velocity.y *= scalar_xy;
            desired_speed_xy = pos_control->get_speed_xy();
        }

        // limit desired velocity to be between maximum climb and descent rates
        desired_velocity.z = constrain_float(desired_velocity.z, -fabsf(pos_control->get_speed_down()), pos_control->get_speed_up());

        // unit vector towards target
        Vector3f dir_to_target_neu = dist_vec_to_target_neu;
        const float dir_to_target_neu_len = dir_to_target_neu.length();
        if (!is_zero(dir_to_target_neu_len)) {
            dir_to_target_neu /= dir_to_target_neu_len;
        }

        // create horizontal desired velocity vector (required for slow down calculations)
        Vector2f desired_velocity_xy(desired_velocity.x, desired_velocity.y);

        // create horizontal unit vector towards target (required for slow down calculations)
        Vector2f dir_to_target_xy(desired_velocity_xy.x, desired_velocity_xy.y);
        if (!dir_to_target_xy.is_zero()) {
            dir_to_target_xy.normalize();
        }

        // slow down horizontally as we approach target (use 1/2 of maximum deceleration for gentle slow down)
        const float dist_to_target_xy = Vector2f(dist_vec_to_target_neu.x, dist_vec_to_target_neu.y).length();
        _copter.avoid.limit_velocity(pos_control->get_pos_xy_p().kP().get(), pos_control->get_accel_xy() / 2.0f, desired_velocity_xy, dir_to_target_xy, dist_to_target_xy, _copter.G_Dt);

        // limit the horizontal velocity to prevent fence violations
        _copter.avoid.adjust_velocity(pos_control->get_pos_xy_p().kP().get(), pos_control->get_accel_xy(), desired_velocity_xy, G_Dt);

        // copy horizontal velocity limits back to 3d vector
        desired_velocity.x = desired_velocity_xy.x;
        desired_velocity.y = desired_velocity_xy.y;

        // limit vertical desired_velocity to slow as we approach target (we use 1/2 of maximum deceleration for gentle slow down)
        const float des_vel_z_max = _copter.avoid.get_max_speed(pos_control->get_pos_z_p().kP().get(), pos_control->get_accel_z() / 2.0f, fabsf(dist_vec_to_target_neu.z), _copter.G_Dt);
        desired_velocity.z = constrain_float(desired_velocity.z, -des_vel_z_max, des_vel_z_max);

        // get avoidance adjusted climb rate
        desired_velocity.z = get_avoidance_adjusted_climbrate(desired_velocity.z);
    }

    // re-use guided mode's velocity controller (takes NEU)
    Copter::ModeGuided::set_velocity(desired_velocity);

    Copter::ModeGuided::run();
}

