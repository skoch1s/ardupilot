#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

#include <AP_Stats/AP_Stats.h>              // statistics library

// Code to detect landing for VTOL aircraft in rotorcraft mode
// Adapted from ArduCopter for QuadPlane operations
#define LAND_CHECK_ANGLE_ERROR_DEG  30.0f       // maximum angle error to be considered landing
#define LAND_CHECK_LARGE_ANGLE_RAD  radians(15.0f)     // maximum angle target to be considered landing
#define LAND_CHECK_ACCEL_MOVING     3.0f        // maximum acceleration after subtracting gravity
#define LAND_DETECTOR_TRIGGER_SEC   1.0f        // default trigger time for landing detection
#define LAND_AIRMODE_DETECTOR_TRIGGER_SEC 4.0f  // trigger time when airmode is enabled
#define LAND_DETECTOR_MAYBE_TRIGGER_SEC 0.2f    // trigger time for 'maybe landed'
#define LAND_DETECTOR_ACCEL_MAX     2.0f        // maximum acceleration for stationary detection (m/s^2)
#define LAND_DETECTOR_VEL_Z_MAX     0.3f        // maximum vertical velocity for landing (m/s)
#define LAND_RANGEFINDER_MIN_ALT_M  2.0f        // rangefinder altitude threshold for landing detection

// Logging flag enumeration for VTOL landing detector
// This matches ArduCopter's implementation for consistency
enum class LandDetectorLoggingFlag : uint16_t {
    MOTOR_AT_LOWER_LIMIT = (1U << 0),
    THROTTLE_MIX_AT_MIN = (1U << 1),
    LARGE_ANGLE_REQUEST = (1U << 2),
    LARGE_ANGLE_ERROR = (1U << 3),
    ACCEL_STATIONARY = (1U << 4),
    DESCENT_RATE_LOW = (1U << 5),
    RANGEFINDER_BELOW_2M = (1U << 6),
    WOW = (1U << 7),
    LANDED = (1U << 8),
    LANDED_MAYBE = (1U << 9),
    LANDING = (1U << 10),
    STANDBY_ACTIVE = (1U << 11),
};

// counter to verify landings
static uint32_t land_detector_count = 0;

// run land and crash detectors
// called at MAIN_LOOP_RATE (typically 400Hz)
void QuadPlane::update_land_and_crash_detectors()
{
    // only run landing detector in VTOL modes
    if (!in_vtol_mode() || !available()) {
        land_detector_count = 0;
        return;
    }

    // update 1hz filtered acceleration
    Vector3f accel_ef = plane.ahrs.get_accel_ef();
    accel_ef.z += GRAVITY_MSS;
    land_accel_ef_filter.apply(accel_ef, plane.scheduler.get_loop_period_s());

    update_land_detector();

    // Note: crash detection is handled separately in quadplane.cpp
}

// update_land_detector - checks if we have landed and updates the land_complete flag
// called at MAIN_LOOP_RATE
void QuadPlane::update_land_detector()
{
    // land detector can not use the following sensors because they are unreliable during landing
    // barometer altitude :                 ground effect can cause errors larger than 4m
    // EKF vertical velocity or altitude :  poor barometer and large acceleration from ground impact
    // earth frame angle or angle error :   landing on an uneven surface will force the airframe to match the ground angle
    // gyro output :                        on uneven surface the airframe may rock back an forth after landing
    // range finder :                       tend to be problematic at very short distances
    // input throttle :                     in slow land the input throttle may be only slightly less than hover

#if HAL_LOGGING_ENABLED
    uint16_t logging_flags = 0;
#define SET_LOG_FLAG(condition, flag) if (condition) { logging_flags |= (uint16_t)flag; }
#else
#define SET_LOG_FLAG(condition, flag)
#endif

    if (!motors->armed()) {
        // if disarmed, always landed.
        set_land_complete(true);
    } else if (land_complete) {
        // if throttle output is high then clear landing flag
        if (motors->get_throttle_out() > motors->get_throttle_hover() * 0.5f && 
            motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
            set_land_complete(false);
        }
    } else {
        // not yet landed, check landing criteria

        float land_trigger_sec = LAND_DETECTOR_TRIGGER_SEC;

        // check that the average throttle output is near minimum (motor at lower limit)
        bool motor_at_lower_limit = motors->limit.throttle_lower;
        bool throttle_mix_at_min = attitude_control->is_throttle_mix_min();
        
        SET_LOG_FLAG(motor_at_lower_limit, LandDetectorLoggingFlag::MOTOR_AT_LOWER_LIMIT);
        SET_LOG_FLAG(throttle_mix_at_min, LandDetectorLoggingFlag::THROTTLE_MIX_AT_MIN);

        uint8_t land_detector_scalar = 1;
#if AP_LANDINGGEAR_ENABLED
        if (plane.g2.landing_gear.get_wow_state() != AP_LandingGear::LG_WOW_UNKNOWN) {
            // we have a WoW sensor so lets loosen the strictness of the landing detector
            land_detector_scalar = 2;
        }
#endif

        // check for aggressive flight requests - requested roll or pitch angle below 15 degrees
        const Vector3f& angle_target_rad = attitude_control->get_att_target_euler_rad();
        bool large_angle_request = angle_target_rad.xy().length_squared() > sq(LAND_CHECK_LARGE_ANGLE_RAD);
        SET_LOG_FLAG(large_angle_request, LandDetectorLoggingFlag::LARGE_ANGLE_REQUEST);

        // check for large external disturbance - angle error over 30 degrees
        const float angle_error = attitude_control->get_att_error_angle_deg();
        bool large_angle_error = (angle_error > LAND_CHECK_ANGLE_ERROR_DEG);
        SET_LOG_FLAG(large_angle_error, LandDetectorLoggingFlag::LARGE_ANGLE_ERROR);

        // check that the airframe is not accelerating (not falling or braking after fast forward flight)
        bool accel_stationary = (land_accel_ef_filter.get().length() <= LAND_DETECTOR_ACCEL_MAX * land_detector_scalar);
        SET_LOG_FLAG(accel_stationary, LandDetectorLoggingFlag::ACCEL_STATIONARY);

        // check that vertical speed is within threshold of zero
        float vel_d_ms = 0;
        UNUSED_RESULT(plane.ahrs.get_velocity_D(vel_d_ms));
        const bool descent_rate_low = fabsf(vel_d_ms) < LAND_DETECTOR_VEL_Z_MAX * land_detector_scalar;
        SET_LOG_FLAG(descent_rate_low, LandDetectorLoggingFlag::DESCENT_RATE_LOW);

        // if we have a healthy rangefinder only allow landing detection below 2 meters
#if AP_RANGEFINDER_ENABLED
        bool rangefinder_check = (!plane.rangefinder_use(RangeFinderUse::TAKEOFF_LANDING) || 
                                   plane.relative_ground_altitude(RangeFinderUse::TAKEOFF_LANDING) < LAND_RANGEFINDER_MIN_ALT_M);
#else
        bool rangefinder_check = true;
#endif
        SET_LOG_FLAG(rangefinder_check, LandDetectorLoggingFlag::RANGEFINDER_BELOW_2M);

        // if we have weight on wheels (WoW) or ambiguous unknown. never no WoW
#if AP_LANDINGGEAR_ENABLED
        const bool WoW_check = (plane.g2.landing_gear.get_wow_state() == AP_LandingGear::LG_WOW || 
                                 plane.g2.landing_gear.get_wow_state() == AP_LandingGear::LG_WOW_UNKNOWN);
#else
        const bool WoW_check = true;
#endif
        SET_LOG_FLAG(WoW_check, LandDetectorLoggingFlag::WOW);

        if (motor_at_lower_limit && throttle_mix_at_min && !large_angle_request && !large_angle_error && 
            accel_stationary && descent_rate_low && rangefinder_check && WoW_check) {
            // landed criteria met - increment the counter and check if we've triggered
            if (land_detector_count < land_trigger_sec * plane.scheduler.get_loop_rate_hz()) {
                land_detector_count++;
            } else {
                set_land_complete(true);
            }
        } else {
            // we've sensed movement up or down so reset land_detector
            land_detector_count = 0;
        }
    }

    set_land_complete_maybe(land_complete || (land_detector_count >= LAND_DETECTOR_MAYBE_TRIGGER_SEC * plane.scheduler.get_loop_rate_hz()));

#if HAL_LOGGING_ENABLED
    // @LoggerMessage: LDET
    // @Description: VTOL Land Detector State
    // @Field: TimeUS: Time since system startup
    // @Field: Flags: boolean state flags
    // @FieldBitmaskEnum: Flags: LandDetectorLoggingFlag
    // @Field: Count: landing_detector pass count
    SET_LOG_FLAG(land_complete, LandDetectorLoggingFlag::LANDED);
    SET_LOG_FLAG(land_complete_maybe, LandDetectorLoggingFlag::LANDED_MAYBE);
    Log_LDET(logging_flags, land_detector_count);
#undef SET_LOG_FLAG
#endif
}

#if HAL_LOGGING_ENABLED
void QuadPlane::Log_LDET(uint16_t logging_flags, uint32_t detector_count)
{
    // do not log if no change:
    if (logging_flags == landing_detect.last_logged_flags &&
        detector_count == landing_detect.last_logged_count) {
        return;
    }
    // do not log more than 50Hz:
    const auto now = AP_HAL::millis();
    if (now - landing_detect.last_logged_ms < 20) {
        return;
    }

    landing_detect.last_logged_count = detector_count;
    landing_detect.last_logged_flags = logging_flags;
    landing_detect.last_logged_ms = now;

    AP::logger().WriteStreaming(
        "LDET",
        "TimeUS," "Flags," "Count",
        "s"       "-"      "-",
        "F"       "-"      "-",
        "Q"       "H"      "I",
        AP_HAL::micros64(),
        logging_flags,
        land_detector_count
    );
}
#endif

// set land_complete flag and disarm motors if disarm-on-land is configured
void QuadPlane::set_land_complete(bool b)
{
    // if no change, exit immediately
    if (land_complete == b) {
        return;
    }

    land_detector_count = 0;

#if HAL_LOGGING_ENABLED
    if (b) {
        AP::logger().Write_Event(LogEvent::LAND_COMPLETE);
    } else {
        AP::logger().Write_Event(LogEvent::NOT_LANDED);
    }
#endif
    land_complete = b;

#if AP_STATS_ENABLED
    AP::stats()->set_flying(!b);
#endif

    // tell AHRS flying state
    // set_likely_flying(!b);

    if (!b) {
        // not landed, no further action
        return;
    }

    // landed; trigger disarm-on-land for manual modes
    if (!motors->armed()) {
        // we are not currently armed, so we don't need to disarm
        return;
    }

    // if throttle is not zero, exit immediately
    if (!plane.channel_throttle->in_min_dz()) {
        return;
    }

    // only disarm in manual VTOL modes (QSTABILIZE, QHOVER, QLOITER, QACRO)
    // For AUTO modes, landing is handled by the mission state machine
    if (plane.control_mode == &plane.mode_qstabilize ||
        plane.control_mode == &plane.mode_qhover ||
        plane.control_mode == &plane.mode_qloiter ||
        plane.control_mode == &plane.mode_qacro) {
        
        // Check if auto-disarm is enabled (using quadplane parameter if available)
        // For now, we disarm after landing in manual modes
        plane.arming.disarm(AP_Arming::Method::LANDED);
    }
}

// set land complete maybe flag
void QuadPlane::set_land_complete_maybe(bool b)
{
    // if no change, exit immediately
    if (land_complete_maybe == b) {
        return;
    }

    if (b) {
        LOGGER_WRITE_EVENT(LogEvent::LAND_COMPLETE_MAYBE);
    }
    land_complete_maybe = b;
}

#endif  // HAL_QUADPLANE_ENABLED
