#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    PROTOCOL_REQUEST_NONE = 0,
    PROTOCOL_REQUEST_CALIBRATE,
    PROTOCOL_REQUEST_DISARM,
    PROTOCOL_REQUEST_SETPOINT,
    PROTOCOL_REQUEST_SET_KD,
    PROTOCOL_REQUEST_INVALID_KD,
    PROTOCOL_REQUEST_RESULT,
    PROTOCOL_REQUEST_INVALID
} protocol_request_type_t;

#define PROTOCOL_RESULT_VALUES 4

typedef struct
{
    protocol_request_type_t type;
    float value_mm_s;
    float gain_kd;
    /* 0 for the calibration summary, 'h' or 'f' for one gate's edges. */
    char result_page;
} protocol_request_t;

typedef enum
{
    PROTOCOL_RESPONSE_NONE = 0,
    PROTOCOL_RESPONSE_CALIBRATING,
    PROTOCOL_RESPONSE_DISARMED,
    PROTOCOL_RESPONSE_ACK,
    PROTOCOL_RESPONSE_KD,
    PROTOCOL_RESPONSE_ERR_CAL,
    PROTOCOL_RESPONSE_ERR_DIS,
    PROTOCOL_RESPONSE_ERR_LIM,
    PROTOCOL_RESPONSE_ERR_HRD,
    PROTOCOL_RESPONSE_ERR_EST,
    PROTOCOL_RESPONSE_ERR_POS,
    PROTOCOL_RESPONSE_ERR_COM,
    PROTOCOL_RESPONSE_ERR_MOT,
    PROTOCOL_RESPONSE_ERR_CAN,
    PROTOCOL_RESPONSE_ERR_CMD,
    PROTOCOL_RESPONSE_ERR_KD,
    PROTOCOL_RESPONSE_ERR_SYS,
    PROTOCOL_RESPONSE_RESULT
} protocol_response_type_t;

typedef struct
{
    protocol_response_type_t type;
    float position_mm;
    float velocity_mm_s;
    float acceleration_mm_s2;
    float gain_kd;
    char result_page;
    uint8_t result_phase;
    uint8_t result_fail;
    /* NAN marks a value the calibration has not measured. */
    float result_mm[PROTOCOL_RESULT_VALUES];
} protocol_response_t;

/** Parses at most one complete host request without blocking. */
bool protocol_read_request(protocol_request_t *request);

/** Queues exactly one newline-terminated response. */
bool protocol_write_response(const protocol_response_t *response);

#endif
