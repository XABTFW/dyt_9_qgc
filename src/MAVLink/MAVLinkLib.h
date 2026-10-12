/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#pragma once

#include <stdint.h>

#define HAVE_MAVLINK_CHANNEL_T
#ifdef HAVE_MAVLINK_CHANNEL_T
typedef enum : uint8_t {
    MAVLINK_COMM_0,
    MAVLINK_COMM_1,
    MAVLINK_COMM_2,
    MAVLINK_COMM_3,
    MAVLINK_COMM_4,
    MAVLINK_COMM_5,
    MAVLINK_COMM_6,
    MAVLINK_COMM_7,
    MAVLINK_COMM_8,
    MAVLINK_COMM_9,
    MAVLINK_COMM_10,
    MAVLINK_COMM_11,
    MAVLINK_COMM_12,
    MAVLINK_COMM_13,
    MAVLINK_COMM_14,
    MAVLINK_COMM_15
} mavlink_channel_t;
#endif

#define MAVLINK_COMM_NUM_BUFFERS 16
#define MAVLINK_MAX_SIGNING_STREAMS MAVLINK_COMM_NUM_BUFFERS

#include <mavlink_types.h>

// Pin the project-specific DYT entries here as well as in the generated
// dialect, so the byte parser always validates their CRC extras before Vehicle
// decodes the payloads.
#define MAVLINK_GET_MSG_ENTRY
static inline const mavlink_msg_entry_t* mavlink_get_msg_entry(uint32_t msgid);

#define MAVLINK_EXTERNAL_RX_STATUS
#ifdef MAVLINK_EXTERNAL_RX_STATUS
    extern mavlink_status_t m_mavlink_status[MAVLINK_COMM_NUM_BUFFERS];
#endif

#define MAVLINK_GET_CHANNEL_STATUS
#ifdef MAVLINK_GET_CHANNEL_STATUS
    extern mavlink_status_t* mavlink_get_channel_status(uint8_t chan);
#endif

// #define MAVLINK_NO_SIGN_PACKET
// #define MAVLINK_NO_SIGNATURE_CHECK
#define MAVLINK_USE_MESSAGE_INFO

#include <stddef.h>

// Ignore warnings from mavlink headers for both GCC/Clang and MSVC
#ifdef __GNUC__
#   if __GNUC__ > 8
#       pragma GCC diagnostic push
#       pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#   else
#       pragma GCC diagnostic push
#       pragma GCC diagnostic ignored "-Wall"
#   endif
#else
#   pragma warning(push, 0)
#endif

#include <mavlink.h>

static inline const mavlink_msg_entry_t* mavlink_get_msg_entry(uint32_t msgid)
{
    static const mavlink_msg_entry_t dytEntries[] = {
        { 12925, 169, 7, 7, 3, 4, 5 },
        { 12926, 0, 91, 91, 0, 0, 0 },
        { 12927, 157, 83, 83, 0, 0, 0 },
        { 12928, 227, 29, 29, 0, 0, 0 },
        { 12929, 172, 21, 21, 0, 0, 0 },
        { 12935, 183, 14, 14, 3, 12, 13 },
        { 12936, 26, 11, 11, 3, 8, 9 },
    };

    for (const mavlink_msg_entry_t& entry : dytEntries) {
        if (entry.msgid == msgid) {
            return &entry;
        }
    }

    static const mavlink_msg_entry_t entries[] = MAVLINK_MESSAGE_CRCS;
    uint32_t low = 0;
    uint32_t high = static_cast<uint32_t>(sizeof(entries) / sizeof(entries[0]) - 1);

    while (low < high) {
        const uint32_t mid = (low + 1 + high) / 2;

        if (msgid < entries[mid].msgid) {
            high = mid - 1;
        } else if (msgid > entries[mid].msgid) {
            low = mid;
        } else {
            low = mid;
            break;
        }
    }

    return entries[low].msgid == msgid ? &entries[low] : nullptr;
}

#ifdef __GNUC__
#	pragma GCC diagnostic pop
#else
#	pragma warning(pop, 0)
#endif
