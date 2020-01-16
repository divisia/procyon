/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_procyon.cpp - ODTU otopilot yarismasi kodu
 */

#define LOG_PROCYON ENABLED                  // debugging for this mode

#define GENERIC_YAW_RATE 2000.0f             // yaw rate to use in all mission steps [-4500 ~ 4500]
#define circle_radius 300                    // cm
#define circle_center_x 0                    // distance cm from arm point in x axis
#define circle_center_z 500                  // distance cm from arm point in z axis
#define step_total 60                        // total steps to draw a circle
#define step_angle (360 / step_total)        // angle between steps
double bearing_rad;                          // bearing at the time mode activated, in radians

Vector3f updated_circle_step = Vector3f(0.0f, 0.0f, 200.0f);   // new step to go in circle
uint8_t step_n = 1;                                            // current step number
uint8_t time_mycallback = 0;


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
        case Procyon_BalletMove:
            hypotenuseMove_init();
            break;
        case Procyon_HypotenuseMove:
            dejaVuDescent_init();
            break;
        case Procyon_DejaVuDescent:
            fireHole_init();
            break;
        case Procyon_FireHole:
            // todo: do something cool - closing
            break;
        }
    }

    switch (procyon_state) {
    case Procyon_StopAboveArm:
        stopAboveArm_run();
        break;
    case Procyon_BalletMove:
        balletMove_run();
        break;
    case Procyon_HypotenuseMove:
        hypotenuseMove_run();
        break;
    case Procyon_DejaVuDescent:
        dejaVuDescent_run();
        break;
    case Procyon_FireHole:
        fireHole_run();
        break;
    }
}


// position vector correction
static void position_relative_to_initial_bearing(Vector3f &d_pos) {
    float x = d_pos.x;
    float y = d_pos.y;
    float z = d_pos.z;

    d_pos.x = x * (float)cos(bearing_rad) - y * (float)sin(bearing_rad);
    d_pos.y = y * (float)cos(bearing_rad) + x * (float)sin(bearing_rad);

    time_mycallback+5;
}

static void stopAboveArm_init() {

    procyon_state = Procyon_StopAboveArm;
    procyon_state_complete = false;

    bearing_rad = float(ahrs.yaw_sensor / 100.0f) * DEG_TO_RAD;

    set_auto_yaw_mode(AUTO_YAW_HOLD);
    wp_nav.set_fast_waypoint(false);

    wp_nav.wp_and_spline_init();
    Vector3f destination = (Vector3f(0.0f, 0.0f, 200.0f)); // 200 cm above arm point
    position_relative_to_initial_bearing(destination);

    wp_nav.set_wp_destination(destination);
}

static void stopAboveArm_run() {

    wp_nav.update_wpnav();
    pos_control.update_z_controller();

    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), 0.0f);
    procyon_state_complete = wp_nav.reached_wp_destination();
}


static void balletMove_init() {
    procyon_state_complete = false;
    procyon_state = Procyon_BalletMove;

    // stop by 500 cm ahead (old dest: 0-0-200)
    Vector3f destination = Vector3f(500.0f, 0.0f, 200.0f);
    position_relative_to_initial_bearing(destination);

    wp_nav.set_wp_destination(destination);
}

static void balletMove_run() {
    wp_nav.update_wpnav();
    pos_control.update_z_controller();

    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), GENERIC_YAW_RATE);

    procyon_state_complete = wp_nav.reached_wp_destination();
}


static void hypotenuseMove_init(){
    procyon_state_complete = false;
    procyon_state = Procyon_HypotenuseMove;

    Vector3f destination = Vector3f(0.0f, 0.0f, 700.0f); // stop by 700 cm above (old dest: 500-0-200)
    position_relative_to_initial_bearing(destination);
    wp_nav.set_wp_destination(destination);
}

static void hypotenuseMove_run(){
    wp_nav.update_wpnav();
    pos_control.update_z_controller();

    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), GENERIC_YAW_RATE);

    procyon_state_complete = wp_nav.reached_wp_destination();
}


static void dejaVuDescent_init() {
    procyon_state_complete = false;
    procyon_state = Procyon_DejaVuDescent;

    Vector3f destination = Vector3f(0.0f, 0.0f, 200.0f); // get back to st-1 (old dest: 0-0-700)
    position_relative_to_initial_bearing(destination);
    wp_nav.set_wp_destination(destination);
}

static void dejaVuDescent_run() {
    wp_nav.update_wpnav();
    pos_control.update_z_controller();

    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), GENERIC_YAW_RATE);

    procyon_state_complete = wp_nav.reached_wp_destination();

}


static void fireHole_init() {
    procyon_state_complete = false;
    procyon_state = Procyon_FireHole;

    update_circle_step();
    position_relative_to_initial_bearing(updated_circle_step);
    wp_nav.set_wp_destination(updated_circle_step);
}

static void fireHole_run() {

    // update horizontal and vertical controllers
    wp_nav.update_wpnav();
    pos_control.update_z_controller();
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), GENERIC_YAW_RATE);

    // check if step complete
    if (wp_nav.reached_wp_destination()) {
        step_n++;  // increase step number
        if (step_n >= step_total) {  // if steps are complete, return state complete
            procyon_state_complete = true;
            return;
        }
        update_circle_step();
        position_relative_to_initial_bearing(updated_circle_step);
        wp_nav.set_wp_destination(updated_circle_step);
    }

}


static void update_circle_step() {
    int desired_circle_angle = (step_n * step_angle);

#if LOG_PROCYON
    Log_Write_Data((uint8_t)step_n, (int16_t)desired_circle_angle);
#endif

    updated_circle_step.z = sin((desired_circle_angle - 90) * DEG_TO_RAD) * circle_radius + circle_center_z;
    updated_circle_step.x = cos((desired_circle_angle - 90) * DEG_TO_RAD) * circle_radius + circle_center_x;
}

