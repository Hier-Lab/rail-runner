#include "protocol.h"

#include <math.h>
#include <string.h>

#include "uart.h"

static bool parse_setpoint(const char *text, float *value)
{
    const char *cursor = text;
    int sign = 1;
    int whole = 0;
    int fraction = 0;
    int digits = 0;

    if (*cursor == '+' || *cursor == '-')
    {
        sign = *cursor++ == '-' ? -1 : 1;
    }
    while (*cursor >= '0' && *cursor <= '9')
    {
        if (whole > 1000)
        {
            return false;
        }
        whole = whole * 10 + (*cursor++ - '0');
        digits++;
    }
    if (digits == 0)
    {
        return false;
    }
    if (*cursor == '.')
    {
        cursor++;
        if (*cursor < '0' || *cursor > '9')
        {
            return false;
        }
        fraction = *cursor++ - '0';
    }
    if (*cursor != '\0')
    {
        return false;
    }

    *value = (float)(sign * (whole * 10 + fraction)) / 10.0f;
    return true;
}

static bool parse_gain_kd(const char *text, float *value)
{
    const char *cursor = text;
    unsigned int whole = 0U;
    unsigned int fraction = 0U;
    unsigned int scale = 1U;
    unsigned int fraction_digits = 0U;

    if (*cursor < '0' || *cursor > '9')
    {
        return false;
    }
    while (*cursor >= '0' && *cursor <= '9')
    {
        whole = whole * 10U + (unsigned int)(*cursor++ - '0');
        if (whole > 1U)
        {
            return false;
        }
    }
    if (*cursor == '.')
    {
        cursor++;
        while (*cursor >= '0' && *cursor <= '9' && fraction_digits < 3U)
        {
            fraction = fraction * 10U + (unsigned int)(*cursor++ - '0');
            scale *= 10U;
            fraction_digits++;
        }
        if (fraction_digits == 0U)
        {
            return false;
        }
    }
    if (*cursor != '\0' || (whole == 1U && fraction != 0U))
    {
        return false;
    }

    *value = (float)whole + (float)fraction / (float)scale;
    return true;
}

bool protocol_read_request(protocol_request_t *request)
{
    char line[24];
    if (request == 0 || !uart_read_line(line, sizeof(line)))
    {
        return false;
    }

    request->value_mm_s = 0.0f;
    request->gain_kd = 0.0f;
    request->result_page = '\0';
    if (strcmp(line, "cal") == 0)
    {
        request->type = PROTOCOL_REQUEST_CALIBRATE;
    }
    else if (strcmp(line, "dis") == 0)
    {
        request->type = PROTOCOL_REQUEST_DISARM;
    }
    else if (strncmp(line, "sp ", 3) == 0 &&
             parse_setpoint(line + 3, &request->value_mm_s))
    {
        request->type = PROTOCOL_REQUEST_SETPOINT;
    }
    else if (strncmp(line, "kd ", 3) == 0 &&
             parse_gain_kd(line + 3, &request->gain_kd))
    {
        request->type = PROTOCOL_REQUEST_SET_KD;
    }
    else if (strcmp(line, "kd") == 0 || strncmp(line, "kd ", 3) == 0)
    {
        request->type = PROTOCOL_REQUEST_INVALID_KD;
    }
    else if (strcmp(line, "res") == 0)
    {
        request->type = PROTOCOL_REQUEST_RESULT;
    }
    else if (strcmp(line, "res h") == 0 || strcmp(line, "res f") == 0)
    {
        request->type = PROTOCOL_REQUEST_RESULT;
        request->result_page = line[4];
    }
    else
    {
        request->type = PROTOCOL_REQUEST_INVALID;
    }
    return true;
}

/* Replies are built inside the 1 ms control cycle on a 16 MHz core, where each
   snprintf call costs about 2,000 cycles; these cost a few hundred. Values are
   clamped so every line fits the 64-byte buffers. */
#define FORMAT_LIMIT 99999999.0f

static long to_scaled(float value, float scale)
{
    float scaled = value * scale;

    if (isnan(scaled))
    {
        return 0L;
    }
    if (scaled > FORMAT_LIMIT)
    {
        scaled = FORMAT_LIMIT;
    }
    if (scaled < -FORMAT_LIMIT)
    {
        scaled = -FORMAT_LIMIT;
    }
    return (long)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

static char *put_text(char *out, const char *text)
{
    while (*text != '\0')
    {
        *out++ = *text++;
    }
    return out;
}

static char *put_uint(char *out, unsigned long value)
{
    char digits[11];
    int count = 0;

    do
    {
        digits[count++] = (char)('0' + value % 10UL);
        value /= 10UL;
    } while (value != 0UL);
    while (count > 0)
    {
        *out++ = digits[--count];
    }
    return out;
}

static char *put_fraction(char *out, unsigned long value, int places)
{
    for (int divisor_places = places - 1; divisor_places >= 0; divisor_places--)
    {
        unsigned long divisor = 1UL;
        for (int i = 0; i < divisor_places; i++)
        {
            divisor *= 10UL;
        }
        *out++ = (char)('0' + (value / divisor) % 10UL);
    }
    return out;
}

static char *put_scaled(char *out, long scaled, unsigned long divisor, int places)
{
    unsigned long magnitude = scaled < 0L ? (unsigned long)(-scaled) :
                                            (unsigned long)scaled;

    if (scaled < 0L)
    {
        *out++ = '-';
    }
    out = put_uint(out, magnitude / divisor);
    *out++ = '.';
    return put_fraction(out, magnitude % divisor, places);
}

static char *put_tenths(char *out, float value)
{
    return put_scaled(out, to_scaled(value, 10.0f), 10UL, 1);
}

static char *put_hundredths(char *out, float value)
{
    if (isnan(value))
    {
        *out++ = '-';
        return out;
    }
    return put_scaled(out, to_scaled(value, 100.0f), 100UL, 2);
}

/* Terminates the line built from line..end and queues it. */
static bool send_line(char *line, char *end)
{
    *end++ = '\n';
    *end = '\0';
    return uart_write(line);
}

static bool write_result(const protocol_response_t *response)
{
    char line[64];
    char *out = put_text(line, "res ");

    if (response->result_page == '\0')
    {
        out = put_uint(out, response->result_phase);
        *out++ = ' ';
        out = put_uint(out, response->result_fail);
        for (int index = 0; index < 3; index++)
        {
            *out++ = ' ';
            out = put_hundredths(out, response->result_mm[index]);
        }
    }
    else
    {
        *out++ = response->result_page;
        for (int index = 0; index < PROTOCOL_RESULT_VALUES; index++)
        {
            *out++ = ' ';
            out = put_hundredths(out, response->result_mm[index]);
        }
    }
    return send_line(line, out);
}

bool protocol_write_response(const protocol_response_t *response)
{
    static const char *const fixed[] = {
        [PROTOCOL_RESPONSE_CALIBRATING] = "cal\n",
        [PROTOCOL_RESPONSE_DISARMED] = "dis\n",
        [PROTOCOL_RESPONSE_ERR_CAL] = "err cal\n",
        [PROTOCOL_RESPONSE_ERR_DIS] = "err dis\n",
        [PROTOCOL_RESPONSE_ERR_LIM] = "err lim\n",
        [PROTOCOL_RESPONSE_ERR_EST] = "err est\n",
        [PROTOCOL_RESPONSE_ERR_POS] = "err pos\n",
        [PROTOCOL_RESPONSE_ERR_COM] = "err com\n",
        [PROTOCOL_RESPONSE_ERR_MOT] = "err mot\n",
        [PROTOCOL_RESPONSE_ERR_CAN] = "err can\n",
        [PROTOCOL_RESPONSE_ERR_CMD] = "err cmd\n",
        [PROTOCOL_RESPONSE_ERR_KD] = "err kd\n",
        [PROTOCOL_RESPONSE_ERR_SYS] = "err sys\n",
    };
    char line[64];

    if (response == 0 || response->type == PROTOCOL_RESPONSE_NONE)
    {
        return true;
    }
    if (response->type == PROTOCOL_RESPONSE_ACK)
    {
        char *out = put_text(line, "ack ");
        out = put_tenths(out, response->position_mm);
        *out++ = ' ';
        out = put_tenths(out, response->velocity_mm_s);
        *out++ = ' ';
        out = put_tenths(out, response->acceleration_mm_s2);
        return send_line(line, out);
    }
    if (response->type == PROTOCOL_RESPONSE_RESULT)
    {
        return write_result(response);
    }
    if (response->type == PROTOCOL_RESPONSE_KD)
    {
        unsigned long thousandths =
            (unsigned long)(response->gain_kd * 1000.0f + 0.5f);
        char *out = put_text(line, "kd ");
        out = put_uint(out, thousandths / 1000UL);
        *out++ = '.';
        out = put_fraction(out, thousandths % 1000UL, 3);
        return send_line(line, out);
    }
    if ((unsigned int)response->type >=
        sizeof(fixed) / sizeof(fixed[0]) || fixed[response->type] == 0)
    {
        return false;
    }
    return uart_write(fixed[response->type]);
}
