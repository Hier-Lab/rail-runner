#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>

#define CAL_PROBE_VEL 2.0f
#define CAL_SLOW_VEL 2.0f
#define CAL_BACKOFF_VEL 5.0f
/* Only used between the two gates and for parking. Everything between a gate
   and its switch, and the blind probe (which may start inside that zone), runs
   at CAL_SLOW_VEL / CAL_PROBE_VEL no matter what the run believes about where
   the gates and switches are. */
#define CAL_FAST_VEL 20.0f

/* Gate-to-switch distances measured on the rig (outer gate edge to switch
   press); every calibration re-measures them, these only size the budgets. */
#define CAL_HOME_ZONE_MM 60.0f
#define CAL_FAR_ZONE_MM 48.0f
/* Slow travel allowed before a switch is declared missing: 1.5 x the longer zone. */
#define CAL_ZONE_BUDGET_MM 90.0f
/* Slow travel toward home before the carriage is known not to be between the
   home gate and the home switch: the home zone plus 20 mm. */
#define CAL_PROBE_BUDGET_MM 80.0f
#define CAL_BACKOFF_BUDGET_MM 10.0f
/* Fast travel allowed before a gate is declared missing: the measured travel
   (324.9 mm) plus margin. */
#define CAL_TRAVEL_BUDGET_MM 360.0f
/* A switch stays exempt from faulting until the carriage is this far off it. */
#define CAL_SWITCH_CLEARANCE_MM 2.0f
#define CAL_PARK_INSIDE_FAR_GATE_MM 10.0f
/* A carriage commanded faster than CAL_STALL_MIN_COMMAND that moves at less
   than CAL_STALL_RATIO of it for this long is against something. */
#define CAL_STALL_MS 500U
#define CAL_STALL_MIN_COMMAND 1.0f
#define CAL_STALL_RATIO 0.25f

/* Numbered as in the calibration proposal. Phase 5 (slow out of the home
   zone) is not run: leaving a switch is safe at any speed. */
typedef enum
{
    CAL_PHASE_IDLE = 0,
    CAL_PHASE_PROBE_HOME = 1,
    CAL_PHASE_FAST_HOME = 2,
    CAL_PHASE_SLOW_HOME = 3,
    CAL_PHASE_BACKOFF_HOME = 4,
    CAL_PHASE_FAST_FAR = 6,
    CAL_PHASE_SLOW_FAR = 7,
    CAL_PHASE_BACKOFF_FAR = 8,
    CAL_PHASE_PARK = 9,
    CAL_PHASE_DONE = 10
} cal_phase_t;

typedef enum
{
    CAL_FAIL_NONE = 0,
    CAL_FAIL_DISTANCE = 1,
    CAL_FAIL_TIME = 2,
    CAL_FAIL_STALL = 3
} cal_fail_t;

typedef enum
{
    CAL_RUNNING = 0,
    CAL_DONE,
    CAL_FAILED
} cal_status_t;

typedef enum
{
    CAL_SIGNAL_HOME_SWITCH = 0,
    CAL_SIGNAL_FAR_SWITCH,
    CAL_SIGNAL_HOME_GATE,
    CAL_SIGNAL_FAR_GATE,
    CAL_SIGNAL_COUNT
} cal_signal_id_t;

typedef struct
{
    bool active;
    /* Position at the first sample of the latest confirmed change. */
    float edge_mm;
} cal_signal_t;

typedef struct
{
    cal_signal_t signal[CAL_SIGNAL_COUNT];
    float position_mm;
    float velocity_mm_s;
    float command_velocity_mm_s;
    float command_acceleration_mm_s2;
    uint32_t now_ms;
} cal_inputs_t;

/* "In" is travel toward the gate's own end, "out" is travel away from it.
   The leading edge is the one met first on the way in. */
typedef enum
{
    CAL_EDGE_LEAD_IN = 0,
    CAL_EDGE_TRAIL_IN,
    CAL_EDGE_TRAIL_OUT,
    CAL_EDGE_LEAD_OUT,
    CAL_EDGE_COUNT
} cal_edge_t;

/* Positions are measured from the calibrated zero and are NAN until seen. */
typedef struct
{
    cal_phase_t phase;
    cal_fail_t fail;
    float travel_mm;
    float far_release_mm;
    float home_press_mm;
    float home_gate_mm[CAL_EDGE_COUNT];
    float far_gate_mm[CAL_EDGE_COUNT];
} cal_result_t;

/** Starts a calibration run from the current sensor states. */
void cal_start(const cal_inputs_t *inputs);

/** Advances one control cycle and sets the velocity to request. */
cal_status_t cal_step(const cal_inputs_t *inputs,
                      float *requested_velocity_mm_s);

/** True while the running phase is meant to touch the home switch. */
bool cal_home_switch_expected(float position_mm);

/** True while the running phase is meant to touch the far switch. */
bool cal_far_switch_expected(float position_mm);

/** Position of the calibrated zero in the frame the inputs were given in. */
float cal_zero_mm(void);

/** Reports the latest run, including a failed or still-running one. */
void cal_get_result(cal_result_t *result);

#endif
