#include "calibration.h"

#include <math.h>

#include "motion.h"

#define TIME_BUDGET_SCALE 1.5f
#define TIME_BUDGET_SLACK_MS 3000U

typedef enum
{
    GATE_HOME = 0,
    GATE_FAR,
    GATE_COUNT
} gate_id_t;

/* Every position below is in the frame of the inputs, which stays fixed for
   the whole run; results are converted to the calibrated zero when read. */
static cal_phase_t phase;
static cal_fail_t fail;
static bool was_active[CAL_SIGNAL_COUNT];
static float phase_start_mm;
static uint32_t phase_start_ms;
static uint32_t last_following_ms;
static bool zero_known;
static float zero_mm;
static float home_press_mm;
static float far_press_mm;
static float far_release_mm;
static float gate_mm[GATE_COUNT][CAL_EDGE_COUNT];
static float park_mm;
static bool parking_brake;

static void enter(cal_phase_t next, const cal_inputs_t *inputs)
{
    phase = next;
    phase_start_mm = inputs->position_mm;
    phase_start_ms = inputs->now_ms;
}

static float phase_speed(void)
{
    switch (phase)
    {
        case CAL_PHASE_PROBE_HOME: return CAL_PROBE_VEL;
        case CAL_PHASE_SLOW_HOME:
        case CAL_PHASE_SLOW_FAR: return CAL_SLOW_VEL;
        case CAL_PHASE_BACKOFF_HOME:
        case CAL_PHASE_BACKOFF_FAR: return CAL_BACKOFF_VEL;
        default: return CAL_FAST_VEL;
    }
}

static float phase_velocity(void)
{
    switch (phase)
    {
        case CAL_PHASE_BACKOFF_HOME:
        case CAL_PHASE_FAST_FAR:
        case CAL_PHASE_SLOW_FAR: return phase_speed();
        case CAL_PHASE_PARK: return parking_brake ? 0.0f : -phase_speed();
        default: return -phase_speed();
    }
}

static float phase_budget_mm(void)
{
    switch (phase)
    {
        case CAL_PHASE_PROBE_HOME: return CAL_PROBE_BUDGET_MM;
        case CAL_PHASE_SLOW_HOME:
        case CAL_PHASE_SLOW_FAR: return CAL_ZONE_BUDGET_MM;
        case CAL_PHASE_BACKOFF_HOME:
        case CAL_PHASE_BACKOFF_FAR: return CAL_BACKOFF_BUDGET_MM;
        default: return CAL_TRAVEL_BUDGET_MM;
    }
}

/* A jammed carriage never spends its distance budget, so each phase also
   gets the time that distance should take, with slack for ramps. */
static uint32_t phase_budget_ms(void)
{
    return (uint32_t)(phase_budget_mm() / phase_speed() * 1000.0f *
                      TIME_BUDGET_SCALE) + TIME_BUDGET_SLACK_MS;
}

static void record_gate_edge(gate_id_t gate, bool rising, bool inbound,
                             float edge_mm)
{
    cal_edge_t edge = inbound ?
                      (rising ? CAL_EDGE_LEAD_IN : CAL_EDGE_TRAIL_IN) :
                      (rising ? CAL_EDGE_TRAIL_OUT : CAL_EDGE_LEAD_OUT);
    gate_mm[gate][edge] = edge_mm;
}

static void record_edges(const cal_inputs_t *inputs)
{
    bool toward_far = inputs->command_velocity_mm_s > 0.0f;

    for (int id = 0; id < CAL_SIGNAL_COUNT; id++)
    {
        bool active = inputs->signal[id].active;
        float edge_mm = inputs->signal[id].edge_mm;
        if (active == was_active[id])
        {
            continue;
        }
        was_active[id] = active;

        switch ((cal_signal_id_t)id)
        {
            case CAL_SIGNAL_HOME_SWITCH:
                if (active)
                {
                    home_press_mm = edge_mm;
                }
                break;
            case CAL_SIGNAL_FAR_SWITCH:
                if (active)
                {
                    far_press_mm = edge_mm;
                }
                else
                {
                    far_release_mm = edge_mm;
                }
                break;
            case CAL_SIGNAL_HOME_GATE:
                record_gate_edge(GATE_HOME, active, !toward_far, edge_mm);
                break;
            case CAL_SIGNAL_FAR_GATE:
                record_gate_edge(GATE_FAR, active, toward_far, edge_mm);
                break;
            default:
                break;
        }
    }
}

static void update_park(const cal_inputs_t *inputs)
{
    float speed = -inputs->command_velocity_mm_s;
    float remaining = inputs->position_mm - park_mm;

    /* Written as a negated ">" so an unknown park position brakes at once. */
    if (!parking_brake &&
        !(remaining > speed * CONTROL_DT_S +
                      motion_stopping_distance(
                          speed, -inputs->command_acceleration_mm_s2)))
    {
        parking_brake = true;
    }
    if (parking_brake &&
        motion_is_stopped(inputs->command_velocity_mm_s,
                          inputs->command_acceleration_mm_s2))
    {
        phase = CAL_PHASE_DONE;
    }
}

static bool stalled(const cal_inputs_t *inputs)
{
    float command = inputs->command_velocity_mm_s;

    if (fabsf(command) < CAL_STALL_MIN_COMMAND ||
        inputs->velocity_mm_s * command >= CAL_STALL_RATIO * command * command)
    {
        last_following_ms = inputs->now_ms;
    }
    return inputs->now_ms - last_following_ms > CAL_STALL_MS;
}

void cal_start(const cal_inputs_t *inputs)
{
    fail = CAL_FAIL_NONE;
    last_following_ms = inputs->now_ms;
    zero_known = false;
    zero_mm = 0.0f;
    home_press_mm = NAN;
    far_press_mm = NAN;
    far_release_mm = NAN;
    park_mm = 0.0f;
    parking_brake = false;
    for (int gate = 0; gate < GATE_COUNT; gate++)
    {
        for (int edge = 0; edge < CAL_EDGE_COUNT; edge++)
        {
            gate_mm[gate][edge] = NAN;
        }
    }
    for (int id = 0; id < CAL_SIGNAL_COUNT; id++)
    {
        was_active[id] = inputs->signal[id].active;
    }

    if (inputs->signal[CAL_SIGNAL_HOME_SWITCH].active)
    {
        enter(CAL_PHASE_BACKOFF_HOME, inputs);
    }
    else if (inputs->signal[CAL_SIGNAL_HOME_GATE].active)
    {
        enter(CAL_PHASE_SLOW_HOME, inputs);
    }
    else
    {
        enter(CAL_PHASE_PROBE_HOME, inputs);
    }
}

cal_status_t cal_step(const cal_inputs_t *inputs,
                      float *requested_velocity_mm_s)
{
    bool home_switch = inputs->signal[CAL_SIGNAL_HOME_SWITCH].active;
    bool far_switch = inputs->signal[CAL_SIGNAL_FAR_SWITCH].active;
    bool home_gate = inputs->signal[CAL_SIGNAL_HOME_GATE].active;
    bool far_gate = inputs->signal[CAL_SIGNAL_FAR_GATE].active;
    bool far_gate_was_active = was_active[CAL_SIGNAL_FAR_GATE];

    *requested_velocity_mm_s = 0.0f;
    if (phase == CAL_PHASE_IDLE || fail != CAL_FAIL_NONE)
    {
        return CAL_FAILED;
    }
    if (phase == CAL_PHASE_DONE)
    {
        return CAL_DONE;
    }

    record_edges(inputs);

    switch (phase)
    {
        case CAL_PHASE_PROBE_HOME:
            if (home_switch)
            {
                enter(CAL_PHASE_BACKOFF_HOME, inputs);
            }
            else if (home_gate)
            {
                enter(CAL_PHASE_SLOW_HOME, inputs);
            }
            else if ((far_gate_was_active && !far_gate) ||
                     fabsf(inputs->position_mm - phase_start_mm) >=
                     CAL_PROBE_BUDGET_MM)
            {
                /* Either the far gate was just left behind or the whole
                   home zone was covered without meeting anything, so the
                   home gate is still ahead. */
                enter(CAL_PHASE_FAST_HOME, inputs);
            }
            break;

        case CAL_PHASE_FAST_HOME:
            if (home_gate)
            {
                enter(CAL_PHASE_SLOW_HOME, inputs);
            }
            break;

        case CAL_PHASE_SLOW_HOME:
            if (home_switch)
            {
                enter(CAL_PHASE_BACKOFF_HOME, inputs);
            }
            break;

        case CAL_PHASE_BACKOFF_HOME:
            if (!home_switch)
            {
                zero_mm = inputs->signal[CAL_SIGNAL_HOME_SWITCH].edge_mm;
                zero_known = true;
                enter(CAL_PHASE_FAST_FAR, inputs);
            }
            break;

        case CAL_PHASE_FAST_FAR:
            if (far_gate)
            {
                enter(CAL_PHASE_SLOW_FAR, inputs);
            }
            break;

        case CAL_PHASE_SLOW_FAR:
            if (far_switch)
            {
                enter(CAL_PHASE_BACKOFF_FAR, inputs);
            }
            break;

        case CAL_PHASE_BACKOFF_FAR:
            if (!far_switch)
            {
                park_mm = gate_mm[GATE_FAR][CAL_EDGE_LEAD_IN] -
                          CAL_PARK_INSIDE_FAR_GATE_MM;
                enter(CAL_PHASE_PARK, inputs);
            }
            break;

        case CAL_PHASE_PARK:
            update_park(inputs);
            break;

        default:
            break;
    }

    if (phase == CAL_PHASE_DONE)
    {
        return CAL_DONE;
    }
    if (stalled(inputs))
    {
        fail = CAL_FAIL_STALL;
    }
    else if (fabsf(inputs->position_mm - phase_start_mm) > phase_budget_mm())
    {
        fail = CAL_FAIL_DISTANCE;
    }
    else if (inputs->now_ms - phase_start_ms > phase_budget_ms())
    {
        fail = CAL_FAIL_TIME;
    }
    if (fail != CAL_FAIL_NONE)
    {
        return CAL_FAILED;
    }

    *requested_velocity_mm_s = phase_velocity();
    return CAL_RUNNING;
}

bool cal_home_switch_expected(float position_mm)
{
    if (fail != CAL_FAIL_NONE)
    {
        return false;
    }
    switch (phase)
    {
        case CAL_PHASE_PROBE_HOME:
        case CAL_PHASE_SLOW_HOME:
        case CAL_PHASE_BACKOFF_HOME:
            return true;
        case CAL_PHASE_FAST_FAR:
            return position_mm - zero_mm < CAL_SWITCH_CLEARANCE_MM;
        default:
            return false;
    }
}

bool cal_far_switch_expected(float position_mm)
{
    if (fail != CAL_FAIL_NONE)
    {
        return false;
    }
    switch (phase)
    {
        case CAL_PHASE_SLOW_FAR:
        case CAL_PHASE_BACKOFF_FAR:
            return true;
        case CAL_PHASE_PARK:
            return far_release_mm - position_mm < CAL_SWITCH_CLEARANCE_MM;
        default:
            return false;
    }
}

float cal_zero_mm(void)
{
    return zero_mm;
}

static float from_zero(float raw_mm)
{
    return zero_known ? raw_mm - zero_mm : NAN;
}

void cal_get_result(cal_result_t *result)
{
    result->phase = phase;
    result->fail = fail;
    result->travel_mm = from_zero(far_press_mm);
    result->far_release_mm = from_zero(far_release_mm);
    result->home_press_mm = from_zero(home_press_mm);
    for (int edge = 0; edge < CAL_EDGE_COUNT; edge++)
    {
        result->home_gate_mm[edge] = from_zero(gate_mm[GATE_HOME][edge]);
        result->far_gate_mm[edge] = from_zero(gate_mm[GATE_FAR][edge]);
    }
}
