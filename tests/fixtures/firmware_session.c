// File: firmware_session.c
// Description: Host-only serial adapter around the production firmware session.
// The Python fixture serializes access; no hardware or RTOS code is linked.
#include "connection.h"
#include "initialization.h"
#include <stdint.h>
#include <string.h>

#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT
#endif

static connection_t session;
static uint64_t current_us;
static uint8_t incoming[8192], outgoing[8192];
static size_t incoming_size, outgoing_size;
static unsigned sent_count[256];

/** Return the monotonic clock supplied by the Python serial adapter. */
static uint64_t fixture_now(void)
{
    return current_us;
}

/** Copy a complete firmware frame into the simulated controller-to-host stream. */
static bool fixture_send(uint8_t *bytes, uint8_t length)
{
    if (outgoing_size + length > sizeof(outgoing)) return false;
    memcpy(outgoing + outgoing_size, bytes, length);
    outgoing_size += length;
    ++sent_count[bytes[1]];
    return true;
}

/** Supply the few hardware operations needed to exercise real session framing. */
static uint8_t fixture_handler(controller_command_t *command, protocol_send_fn reply)
{
    if (command->commandType == INIT) {
        uint8_t error = initialization_validate(command);
        if (error != RESPONSE_OK) return error;
        init_response identity = initialization_response();
        reply((uint8_t *)&identity, sizeof(identity));
        return RESPONSE_OK;
    }
    if (command->commandType == STOP_MOTOR) return RESPONSE_OK;
    if (command->commandType == GET_ENCODER_VALUE) {
        uint16_t value = 0xBEEF;
        reply((uint8_t *)&value, sizeof(value));
        return RESPONSE_OK;
    }
    if (command->commandType == GET_PLATFORM_ODOMETRY)
        return RESPONSE_ODOMETRY_NOT_INITIALIZED;
    return RESPONSE_UNKNOWN_COMMAND;
}

/** Reset the simulated board and initialize its actual protocol-v2 connection. */
EXPORT void fixture_reset(uint64_t now_us)
{
    current_us = now_us;
    incoming_size = outgoing_size = 0;
    memset(sent_count, 0, sizeof(sent_count));
    connection_init(&session, fixture_handler, fixture_send, fixture_now);
}

/** Append serial bytes without assuming a write contains exactly one frame. */
EXPORT int fixture_feed(const uint8_t *bytes, size_t length)
{
    if (incoming_size + length > sizeof(incoming)) return 0;
    memcpy(incoming + incoming_size, bytes, length);
    incoming_size += length;
    return 1;
}

/** Run one command-task iteration, including the production receive gating. */
EXPORT void fixture_poll(uint64_t now_us)
{
    current_us = now_us;
    if (incoming_size && connection_can_receive(&session)) {
        size_t frame_size = (size_t)incoming[0] + 1U;
        if (incoming_size >= frame_size) {
            connection_receive(&session, incoming + 1, frame_size - 1U);
            incoming_size -= frame_size;
            memmove(incoming, incoming + frame_size, incoming_size);
        }
    }
    connection_poll(&session);
}

/** Drain up to the requested number of controller bytes, supporting partial reads. */
EXPORT size_t fixture_read(uint8_t *bytes, size_t capacity)
{
    size_t length = outgoing_size < capacity ? outgoing_size : capacity;
    memcpy(bytes, outgoing, length);
    outgoing_size -= length;
    memmove(outgoing, outgoing + length, outgoing_size);
    return length;
}

/** Count emitted message types to distinguish initial and periodic handshakes. */
EXPORT unsigned fixture_sent_count(unsigned command)
{
    return command < 256U ? sent_count[command] : 0;
}
