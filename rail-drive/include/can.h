#ifndef CAN_H
#define CAN_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t id;
    uint8_t len;
    uint8_t data[8];
} can_frame_t;

/** Initializes CAN1 at 1 Mbit/s and enables RX FIFO0 interrupts. */
bool can_init(void);

/** Queues one extended-ID CAN data frame with a DLC from zero through eight. */
bool can_send(uint32_t id, const uint8_t *data, uint8_t len);

/** Copies the newest frame and its monotonically increasing sequence number. */
bool can_read_latest(can_frame_t *frame, uint32_t *sequence);

#endif
