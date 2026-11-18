#include "control.h"

#include <math.h>

#include "ak60.h"
#include "board.h"
#include "calibration.h"
#include "protocol.h"
#include "uart.h"

#define HOST_TIMEOUT_MS 50U
#define FEEDBACK_TIMEOUT_MS 10U
#define OPTICAL_DEBOUNCE_CYCLES 5U
#define TWO_PI 6.28318530718f

typedef enum
{
    STATE_DEACTIVATED = 0,
    STATE_CALIBRATING,
    STATE_ACTIVE,
    STATE_STOPPING,
    STATE_FAULT
} drive_state_t;

typedef struct
{
    bool candidate;
    bool stable;
    uint8_t count;
    float candidate_position_mm;
    /* Position at the first sample of the latest confirmed change. */
    float edge_position_mm;
} debounced_t;

typedef enum
{
    FAULT_NONE = 0,
    FAULT_ESTOP,
    FAULT_SYSTEM,
    FAULT_MOTOR,
    FAULT_CAN,
    FAULT_VELOCITY_LIMIT,
    FAULT_COMMUNICATION
} drive_fault_t;

static volatile bool cycle_pending;
static volatile bool cycle_active;
static volatile bool overrun_pending;
static volatile uint32_t timer_ticks;

static drive_state_t state;
static drive_fault_t fault;
static drive_fault_t stop_fault;
static float requested_velocity_mm_s;
static float command_velocity_mm_s;
static float command_acceleration_mm_s2;
static float position_mm;
static float velocity_mm_s;
static float velocity_gain_kd;
static bool calibration_complete_pending;
static bool calibration_error_pending;
static bool feedback_seen;
static bool last_motor_tx_ok;
static uint32_t feedback_sequence;
static uint32_t last_feedback_tick;
static uint32_t last_host_tick;
static ak60_state_t motor;

static debounced_t home_switch;
static debounced_t far_switch;
static debounced_t gate_home;
static debounced_t gate_far;
static bool beyond_home_gate;
static bool beyond_far_gate;
/* Far switch position; the nominal MAX_POS until a calibration measures it. */
static float travel_mm = MAX_POS;

static protocol_response_type_t fault_response(drive_fault_t reason)
{
    switch (reason)
    {
        case FAULT_ESTOP: return PROTOCOL_RESPONSE_ERR_EST;
        case FAULT_SYSTEM: return PROTOCOL_RESPONSE_ERR_SYS;
        case FAULT_MOTOR: return PROTOCOL_RESPONSE_ERR_MOT;
        case FAULT_CAN: return PROTOCOL_RESPONSE_ERR_CAN;
        case FAULT_VELOCITY_LIMIT: return PROTOCOL_RESPONSE_ERR_LIM;
        case FAULT_COMMUNICATION: return PROTOCOL_RESPONSE_ERR_COM;
        default: return PROTOCOL_RESPONSE_NONE;
    }
}

/* A stop or fault drops any host request and any calibration reply that was
   still waiting to be sent. */
static void drop_pending_motion(void)
{
    requested_velocity_mm_s = 0.0f;
    calibration_complete_pending = false;
    calibration_error_pending = false;
}

static bool state_is_moving(void)
{
    return state == STATE_CALIBRATING || state == STATE_ACTIVE ||
           state == STATE_STOPPING;
}

static void set_fault(drive_fault_t reason)
{
    if (reason == FAULT_NONE)
    {
        return;
    }
    if (fault == FAULT_NONE || reason < fault)
    {
        fault = reason;
    }
    state = STATE_FAULT;
    stop_fault = FAULT_NONE;
    drop_pending_motion();
}

static void begin_stop(drive_fault_t reason)
{
    state = STATE_STOPPING;
    stop_fault = reason;
    drop_pending_motion();
}

static void debounce(debounced_t *input, bool sample)
{
    if (sample == input->candidate)
    {
        if (input->count < OPTICAL_DEBOUNCE_CYCLES)
        {
            input->count++;
        }
    }
    else
    {
        input->candidate = sample;
        input->count = 1U;
        /* Keep the edge position if this interval is confirmed. */
        input->candidate_position_mm = position_mm;
    }
    if (input->count >= OPTICAL_DEBOUNCE_CYCLES &&
        input->stable != input->candidate)
    {
        input->stable = input->candidate;
        input->edge_position_mm = input->candidate_position_mm;
    }
}

static void debounce_init(debounced_t *input, bool sample)
{
    input->candidate = sample;
    input->stable = sample;
    input->count = OPTICAL_DEBOUNCE_CYCLES;
    input->candidate_position_mm = 0.0f;
    input->edge_position_mm = 0.0f;
}

/*
 * Tracks which side of an optical gate the carriage is on. "Beyond" means
 * past the gate, in the slow zone toward that end. The side only changes
 * when the debounced flag leaves the slot, and it is decided by the
 * direction of travel at that moment, so reversing inside the slot and
 * leaving the way you came in is handled correctly.
 */
static void update_gate(debounced_t *gate, bool sample, bool *beyond,
                        float end_sign)
{
    bool was_stable = gate->stable;

    debounce(gate, sample);
    if (was_stable && !gate->stable && velocity_mm_s != 0.0f)
    {
        *beyond = (velocity_mm_s * end_sign) > 0.0f;
    }
}

static float erpm_to_mm_s(int16_t erpm_counts)
{
    float erpm = (float)erpm_counts * 10.0f;
    return MOTOR_DIRECTION * erpm * PITCH /
           (AK60_POLE_PAIRS * AK60_REDUCTION * 60.0f);
}

static float mm_s_to_rad_s(float linear_velocity)
{
    return MOTOR_DIRECTION * linear_velocity * TWO_PI / PITCH;
}

static void consume_feedback(uint32_t now)
{
    can_frame_t frame;
    uint32_t sequence;
    ak60_state_t sample;

    if (!can_read_latest(&frame, &sequence) || sequence == feedback_sequence ||
        !ak60_parse_feedback(&frame, &sample))
    {
        return;
    }

    feedback_sequence = sequence;
    if (sample.error != 0U)
    {
        motor = sample;
        last_feedback_tick = now;
        feedback_seen = true;
        set_fault(FAULT_MOTOR);
        return;
    }

    motor = sample;
    velocity_mm_s = erpm_to_mm_s(sample.velocity_erpm);
    feedback_seen = true;
    last_feedback_tick = now;
}

static bool feedback_fresh(uint32_t now)
{
    return feedback_seen && now - last_feedback_tick < FEEDBACK_TIMEOUT_MS;
}

static bool calibration_preflight_ok(uint32_t now,
                                     protocol_response_type_t *error)
{
    if (!board_estop_ok())
    {
        *error = PROTOCOL_RESPONSE_ERR_EST;
        return false;
    }
    if (!board_hard_limits_ok())
    {
        *error = PROTOCOL_RESPONSE_ERR_EST;
        return false;
    }
    if (!last_motor_tx_ok || !feedback_fresh(now))
    {
        *error = PROTOCOL_RESPONSE_ERR_CAN;
        return false;
    }
    if (motor.error != 0U)
    {
        *error = PROTOCOL_RESPONSE_ERR_MOT;
        return false;
    }
    return true;
}

/* True when the next step plus the jerk-limited stopping distance reaches the
   end of the operating envelope. Velocities and acceleration are taken toward
   that end. */
static bool must_stop_before_end(float remaining_mm, float velocity,
                                 float candidate_velocity,
                                 float candidate_acceleration)
{
    float next_motion = 0.5f * (velocity + candidate_velocity) * CONTROL_DT_S;

    return next_motion + motion_stopping_distance(candidate_velocity,
                                                   candidate_acceleration) >=
           remaining_mm;
}

static float position_safe_target(float target)
{
    /* Gate hard stops: once past an end's gate, motion further toward that
       end commands zero (the normal jerk-limited profile brakes to a halt
       there, no fault). Motion back away from the end is never blocked
       here, and cal bypasses this function entirely. */
    if (beyond_home_gate && target < 0.0f)
    {
        target = 0.0f;
    }
    if (beyond_far_gate && target > 0.0f)
    {
        target = 0.0f;
    }

    float candidate_velocity = command_velocity_mm_s;
    float candidate_acceleration = command_acceleration_mm_s2;
    motion_profile_step(target, &candidate_velocity, &candidate_acceleration);

    /* The home end is the far end seen in a mirrored frame, so both ends share
       one test with the velocities flipped. */
    if (candidate_velocity > 0.0f &&
        !(position_mm >= travel_mm - POSITION_MARGIN && target < 0.0f) &&
        must_stop_before_end(travel_mm - POSITION_MARGIN - position_mm,
                             command_velocity_mm_s, candidate_velocity,
                             candidate_acceleration))
    {
        return 0.0f;
    }
    if (candidate_velocity < 0.0f &&
        !(position_mm <= MIN_POS + POSITION_MARGIN && target > 0.0f) &&
        must_stop_before_end(position_mm - (MIN_POS + POSITION_MARGIN),
                             -command_velocity_mm_s, -candidate_velocity,
                             -candidate_acceleration))
    {
        return 0.0f;
    }
    return target;
}

static bool command_is_outward(float command)
{
    return (command < 0.0f && position_mm <= MIN_POS + POSITION_MARGIN) ||
           (command > 0.0f && position_mm >= travel_mm - POSITION_MARGIN);
}

static void fill_calibration_inputs(cal_inputs_t *inputs, uint32_t now)
{
    const debounced_t *sources[CAL_SIGNAL_COUNT] = {
        [CAL_SIGNAL_HOME_SWITCH] = &home_switch,
        [CAL_SIGNAL_FAR_SWITCH] = &far_switch,
        [CAL_SIGNAL_HOME_GATE] = &gate_home,
        [CAL_SIGNAL_FAR_GATE] = &gate_far,
    };

    for (int id = 0; id < CAL_SIGNAL_COUNT; id++)
    {
        inputs->signal[id].active = sources[id]->stable;
        inputs->signal[id].edge_mm = sources[id]->edge_position_mm;
    }
    inputs->position_mm = position_mm;
    inputs->velocity_mm_s = velocity_mm_s;
    inputs->command_velocity_mm_s = command_velocity_mm_s;
    inputs->command_acceleration_mm_s2 = command_acceleration_mm_s2;
    inputs->now_ms = now;
}

static void start_calibration(uint32_t now)
{
    cal_inputs_t inputs;

    state = STATE_CALIBRATING;
    drop_pending_motion();
    fill_calibration_inputs(&inputs, now);
    cal_start(&inputs);
}

static bool fault_can_clear(uint32_t now)
{
    if (!board_estop_ok() || !board_hard_limits_ok() || board_home_switch_active())
    {
        return false;
    }
    if (fault == FAULT_CAN || fault == FAULT_MOTOR)
    {
        return last_motor_tx_ok && feedback_fresh(now) && motor.error == 0U;
    }
    return true;
}

_Static_assert(CAL_EDGE_COUNT == PROTOCOL_RESULT_VALUES,
               "a res line carries one gate's edges");

static void fill_result_response(char page, protocol_response_t *response)
{
    cal_result_t result;

    cal_get_result(&result);
    response->type = PROTOCOL_RESPONSE_RESULT;
    response->result_page = page;
    response->result_phase = (uint8_t)result.phase;
    response->result_fail = (uint8_t)result.fail;
    for (int edge = 0; edge < CAL_EDGE_COUNT; edge++)
    {
        response->result_mm[edge] = page == 'h' ? result.home_gate_mm[edge] :
                                                  result.far_gate_mm[edge];
    }
    if (page == '\0')
    {
        response->result_mm[0] = result.travel_mm;
        response->result_mm[1] = result.far_release_mm;
        response->result_mm[2] = result.home_press_mm;
    }
}

/* Replies that say nothing about drive health are sent as they are even while
   a fault is latched; every other reply becomes the fault's own error. */
static bool survives_fault(protocol_response_type_t type)
{
    return type == PROTOCOL_RESPONSE_ERR_CAL ||
           type == PROTOCOL_RESPONSE_ERR_LIM ||
           type == PROTOCOL_RESPONSE_ERR_CMD ||
           type == PROTOCOL_RESPONSE_ERR_KD ||
           type == PROTOCOL_RESPONSE_KD ||
           type == PROTOCOL_RESPONSE_RESULT;
}

static void handle_request(const protocol_request_t *request, uint32_t now,
                           protocol_response_t *response)
{
    if (request->type == PROTOCOL_REQUEST_NONE)
    {
        return;
    }

    if (request->type == PROTOCOL_REQUEST_SET_KD)
    {
        velocity_gain_kd = request->gain_kd;
        response->type = PROTOCOL_RESPONSE_KD;
        response->gain_kd = velocity_gain_kd;
        return;
    }
    if (request->type == PROTOCOL_REQUEST_INVALID_KD)
    {
        response->type = PROTOCOL_RESPONSE_ERR_KD;
        return;
    }
    if (request->type == PROTOCOL_REQUEST_RESULT)
    {
        fill_result_response(request->result_page, response);
        return;
    }

    if (fault != FAULT_NONE)
    {
        if (request->type == PROTOCOL_REQUEST_DISARM && fault_can_clear(now))
        {
            fault = FAULT_NONE;
            state = STATE_DEACTIVATED;
            stop_fault = FAULT_NONE;
            command_velocity_mm_s = 0.0f;
            command_acceleration_mm_s2 = 0.0f;
            response->type = PROTOCOL_RESPONSE_DISARMED;
        }
        else
        {
            response->type = fault_response(fault);
        }
        return;
    }

    switch (state)
    {
        case STATE_DEACTIVATED:
            if (request->type == PROTOCOL_REQUEST_DISARM)
            {
                response->type = PROTOCOL_RESPONSE_DISARMED;
            }
            else if (request->type == PROTOCOL_REQUEST_CALIBRATE)
            {
                if (calibration_preflight_ok(now, &response->type))
                {
                    start_calibration(now);
                    response->type = PROTOCOL_RESPONSE_CALIBRATING;
                }
            }
            else
            {
                response->type = PROTOCOL_RESPONSE_ERR_DIS;
            }
            break;

        case STATE_CALIBRATING:
            if (request->type == PROTOCOL_REQUEST_CALIBRATE)
            {
                response->type = PROTOCOL_RESPONSE_CALIBRATING;
            }
            else if (request->type == PROTOCOL_REQUEST_DISARM)
            {
                begin_stop(FAULT_NONE);
                response->type = PROTOCOL_RESPONSE_DISARMED;
            }
            else
            {
                begin_stop(FAULT_NONE);
                response->type = PROTOCOL_RESPONSE_ERR_CAL;
            }
            break;

        case STATE_ACTIVE:
            if (request->type == PROTOCOL_REQUEST_DISARM)
            {
                begin_stop(FAULT_NONE);
                response->type = PROTOCOL_RESPONSE_DISARMED;
            }
            else if (request->type == PROTOCOL_REQUEST_CALIBRATE)
            {
                response->type = PROTOCOL_RESPONSE_ERR_CAL;
            }
            else if (request->type == PROTOCOL_REQUEST_SETPOINT)
            {
                calibration_complete_pending = false;
                if (fabsf(request->value_mm_s) > MAX_VEL)
                {
                    set_fault(FAULT_VELOCITY_LIMIT);
                    response->type = PROTOCOL_RESPONSE_ERR_LIM;
                }
                else
                {
                    last_host_tick = now;
                    if (command_is_outward(request->value_mm_s))
                    {
                        requested_velocity_mm_s = 0.0f;
                        response->type = PROTOCOL_RESPONSE_ERR_POS;
                    }
                    else
                    {
                        requested_velocity_mm_s = request->value_mm_s;
                        response->type = PROTOCOL_RESPONSE_ACK;
                    }
                }
            }
            else
            {
                response->type = PROTOCOL_RESPONSE_ERR_CMD;
            }
            break;

        case STATE_STOPPING:
            response->type = request->type == PROTOCOL_REQUEST_DISARM ?
                             PROTOCOL_RESPONSE_DISARMED :
                             PROTOCOL_RESPONSE_ERR_DIS;
            break;

        case STATE_FAULT:
            response->type = fault_response(fault);
            break;
    }
}

static void update_calibration(uint32_t now)
{
    cal_inputs_t inputs;
    cal_result_t result;

    if (state != STATE_CALIBRATING)
    {
        return;
    }

    fill_calibration_inputs(&inputs, now);
    switch (cal_step(&inputs, &requested_velocity_mm_s))
    {
        case CAL_DONE:
            cal_get_result(&result);
            position_mm -= cal_zero_mm();
            travel_mm = result.travel_mm;
            /* The run parks between the gates. */
            beyond_home_gate = false;
            beyond_far_gate = false;
            state = STATE_ACTIVE;
            calibration_complete_pending = true;
            requested_velocity_mm_s = 0.0f;
            break;
        case CAL_FAILED:
            begin_stop(FAULT_NONE);
            calibration_error_pending = true;
            break;
        default:
            break;
    }
}

static bool send_motor_command(void)
{
    if (state == STATE_DEACTIVATED || state == STATE_FAULT)
    {
        return ak60_send_zero_torque();
    }
    return ak60_send_velocity_rad_s(mm_s_to_rad_s(command_velocity_mm_s),
                                    velocity_gain_kd);
}

static void run_control_cycle(void)
{
    uint32_t now = timer_ticks;
    protocol_request_t request = {PROTOCOL_REQUEST_NONE, 0.0f, 0.0f, '\0'};
    protocol_response_t response = {0};
    bool has_request = protocol_read_request(&request);
    bool was_calibrating = state == STATE_CALIBRATING;

    if (uart_take_rx_overflow() || overrun_pending)
    {
        overrun_pending = false;
        set_fault(FAULT_SYSTEM);
    }
    if (!board_estop_ok())
    {
        set_fault(FAULT_ESTOP);
    }
    /* Both switches are E-stop-class kills, same as the wired E-stop, except
       in the calibration phases that deliberately ride onto them. The home
       switch is also harmless while deactivated: it is where cal starts. */
    if (!board_hard_limits_ok() &&
        !(state == STATE_CALIBRATING && cal_far_switch_expected(position_mm)))
    {
        set_fault(FAULT_ESTOP);
    }
    if (board_home_switch_active() &&
        (state == STATE_ACTIVE || state == STATE_STOPPING ||
         (state == STATE_CALIBRATING &&
          !cal_home_switch_expected(position_mm))))
    {
        set_fault(FAULT_ESTOP);
    }
    consume_feedback(now);
    if (feedback_fresh(now) && motor.error == 0U)
    {
        position_mm += velocity_mm_s * CONTROL_PERIOD_US / 1000000.0f;
    }
    debounce(&home_switch, board_home_switch_active());
    debounce(&far_switch, !board_hard_limits_ok());
    update_gate(&gate_home, board_gate_home_active(), &beyond_home_gate, -1.0f);
    update_gate(&gate_far, board_gate_far_active(), &beyond_far_gate, 1.0f);

    if (state_is_moving() && !feedback_fresh(now))
    {
        set_fault(FAULT_CAN);
    }

    if (has_request)
    {
        handle_request(&request, now, &response);
    }

    if (state == STATE_ACTIVE && requested_velocity_mm_s != 0.0f &&
        now - last_host_tick >= HOST_TIMEOUT_MS)
    {
        /* Only a host that goes silent while still commanding motion is a
           hazard; once it has told us to hold at zero (including right
           after calibration), losing the connection is safe to ignore so
           the carriage can sit idle between host sessions. */
        begin_stop(FAULT_COMMUNICATION);
    }

    update_calibration(now);

    if (state_is_moving())
    {
        float target = state == STATE_ACTIVE ?
                       position_safe_target(requested_velocity_mm_s) :
                       requested_velocity_mm_s;
        motion_profile_step(target, &command_velocity_mm_s,
                            &command_acceleration_mm_s2);
    }

    if (state == STATE_STOPPING &&
        motion_is_stopped(command_velocity_mm_s, command_acceleration_mm_s2))
    {
        if (stop_fault == FAULT_NONE)
        {
            state = STATE_DEACTIVATED;
        }
        else
        {
            drive_fault_t completed_fault = stop_fault;
            stop_fault = FAULT_NONE;
            set_fault(completed_fault);
        }
    }

    last_motor_tx_ok = send_motor_command();
    if (!last_motor_tx_ok)
    {
        set_fault(FAULT_CAN);
    }

    if (calibration_complete_pending &&
        response.type == PROTOCOL_RESPONSE_NONE)
    {
        calibration_complete_pending = false;
        last_host_tick = now;
        response.type = PROTOCOL_RESPONSE_ACK;
    }
    else if (!has_request && was_calibrating && fault != FAULT_NONE)
    {
        response.type = fault_response(fault);
    }
    else if (calibration_error_pending &&
             response.type == PROTOCOL_RESPONSE_NONE)
    {
        calibration_error_pending = false;
        response.type = PROTOCOL_RESPONSE_ERR_CAL;
    }

    if (has_request || response.type != PROTOCOL_RESPONSE_NONE)
    {
        if (fault != FAULT_NONE && !survives_fault(response.type))
        {
            response.type = fault_response(fault);
        }
        response.position_mm = position_mm;
        response.velocity_mm_s = velocity_mm_s;
        response.acceleration_mm_s2 = command_acceleration_mm_s2;
        if (!protocol_write_response(&response))
        {
            set_fault(FAULT_SYSTEM);
        }
    }
}

void control_init(void)
{
    state = STATE_DEACTIVATED;
    velocity_gain_kd = AK60_DEFAULT_KD;
    debounce_init(&home_switch, board_home_switch_active());
    debounce_init(&far_switch, !board_hard_limits_ok());
    debounce_init(&gate_home, board_gate_home_active());
    debounce_init(&gate_far, board_gate_far_active());
    beyond_home_gate = false;
    beyond_far_gate = false;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

bool control_run_pending(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (!cycle_pending)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return false;
    }
    cycle_pending = false;
    cycle_active = true;
    if (primask == 0U)
    {
        __enable_irq();
    }

    run_control_cycle();
    cycle_active = false;
    return true;
}

void control_timer_tick_isr(void)
{
    timer_ticks++;
    if (cycle_pending || cycle_active)
    {
        overrun_pending = true;
    }
    cycle_pending = true;
}
