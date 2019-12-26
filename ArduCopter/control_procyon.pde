/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_procyon.cpp - ODTU otopilot yarismasi kodu
 */

#define circle_radius 300.0 // cm
#define circle_center_x 500.0 // distance cm from arm point
#define circle_center_z 500.0 // distance cm from arm point
#define circle_steps 60 // circle update rate

#define initial_altitude 200.0 // altitude to start autonomous mode in cm

const unsigned int step_angle = (360 / circle_steps); // angle between steps


int circle_next_step_x = -1;
int circle_next_step_z = -1;

 // procyon_init - initialise flight mode
static bool procyon_init(bool ignore_checks)
{
    // if not armed or throtle at zero, deny mode
    if (!motors.armed() || g.rc_3.control_in < 50) {
        return false;
    }
    stopAboveArm_init();
    return true;
}

// procyon_run - runs the main controller
// will be called at 100hz or more
static void procyon_run()
{
    // if not armed or throttle at zero, set throttle to zero and exit immediately
    if (!motors.armed()) {
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.set_throttle_out(0, false);
        return;
    }

    // if state completed, move to next
    if (procyon_state_complete) {
        switch (procyon_state) {

        case Procyon_StopAboveArm:
            balletMove_init();
            break;

        }
    }

    switch (procyon_state_complete) {
    case Procyon_StopAboveArm:
        stopAboveArm_run();
        break;
    }
}

static void stopAboveArm_init() {

    procyon_state = Procyon_StopAboveArm;
    procyon_state_complete = false;

    wp_nav.wp_and_spline_init();
    Vector3f destination = Vector3f(0.0f, 0.0f, 200.0f); // 200 cm above arm point

    wp_nav.set_wp_destination(destination);
    set_auto_yaw_mode(AUTO_YAW_HOLD);
}

static void stopAboveArm_run() {

    /*
    TBD:
    arming check
    landing check

    yaw rate is not needed here.
    */

    wp_nav.update_wpnav();
    pos_control.update_z_controller();

    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), 0.0f);

    procyon_state_complete = wp_nav.reached_wp_destination();
}


static void balletMove_init() {
    procyon_state_complete = false;
    procyon_state = Procyon_BalletMove;
}

static void balletMove_run() {
    wp_nav.update_wpnav();

    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), 1000.0f);
}


static void update_circle_step(unsigned int step) {
    unsigned int desired_circle_angle = step * step_angle;
    circle_next_step_z = sin(desired_circle_angle - 90)*circle_radius + circle_center_z;
    circle_next_step_x = cos(desired_circle_angle - 90)*circle_radius + circle_center_x;
}

