#include "TextMessageModule.h"
#include "MeshService.h"
#ifdef FLAMINGO
#include "MeshTypes.h"
#endif
#include "NodeDB.h"
#include "PowerFSM.h"
#include "buzz.h"
#include "configuration.h"
#ifdef FLAMINGO
#include "RangeTestModule.h"
#endif

#include "graphics/Screen.h"
TextMessageModule *textMessageModule;

#ifdef FLAMINGO
#define MAX_ADMIN_MSG 31

void parseAdmin(pb_size_t size, char *payload, bool is_dm)
{
    char local_payload[MAX_ADMIN_MSG + 1];
    pb_size_t new_size;
    if (size < 4)
        return; // too short to be an ADMIN message

    // Check for Alert Bell emojii, toggle RT if alert bell received
    /// use alert bell only if it is a dm
    if (is_dm && payload[0] == 0xF0 && payload[1] == 0x9F && payload[2] == 0x94 && payload[3] == 0x94) {
        if (getRtDynanmicEnable()) {
            LOG_INFO("Found Alert Bell, Dynamic Rangetest OFF ");
            setRtDynamicEnable(0);
        } else {
            LOG_INFO("Found Alert Bell, Dynamic Rangetest ON ");
            setRtDynamicEnable(1);
        }
        return;
    }

    if (!((payload[0] == 'A' || payload[0] == 'a') && (payload[1] == 'D' || payload[1] == 'd')))
        return;
    // this is an ADMIN message
    char adminBuf[MAX_ADMIN_MSG + 1] = "adrt on xxxx"; // targeted to specific node
    auto node = nodeDB->getMeshNode(myNodeInfo.my_node_num);
    const char *sender = (node) ? node->user.short_name : "????";
    for (int i = 0; i < 4; i++) {
        adminBuf[i + 8] = tolower(sender[i]);
    }
    new_size = (size < MAX_ADMIN_MSG) ? size : MAX_ADMIN_MSG;
    strncpy(local_payload, payload, new_size);
    local_payload[new_size] = '\0';

    for (int i = 0; i < new_size; i++) {
        local_payload[i] = tolower(local_payload[i]);
    }

    if (strcmp(adminBuf, local_payload) == 0) {
        LOG_INFO("Turning Dynamic Rangetest ON targeted at node: %s", sender);
        setRtDynamicEnable(1);
        setRtHop(0);
    } else if (strcmp("adrt on", local_payload) == 0) {
        LOG_INFO("Turning Dynamic Rangetest ON");
        setRtDynamicEnable(1);
        setRtHop(0);
    } else if (strcmp("adrt off", local_payload) == 0) {
        LOG_INFO("Turning Dynamic Rangetest OFF");
        setRtDynamicEnable(0);
    } else if (strncmp("adrt delay", local_payload, 10) == 0 && new_size >= 13) {
        if (strncmp("15", local_payload + 11, 2) == 0) {
            LOG_INFO("Rangetest delay is 15");
            moduleConfig.range_test.sender = 15;
        } else if (strncmp("30", local_payload + 11, 2) == 0) {
            LOG_INFO("Rangetest delay is 30");
            moduleConfig.range_test.sender = 30;
        } else if (strncmp("60", local_payload + 11, 2) == 0) {
            LOG_INFO("Rangetest delay is 60");
            moduleConfig.range_test.sender = 60;
        }
    }
}

#ifdef DEBUG_PORT
#define EMOJI_BUFF_SIZE 400
static char emoji_buf[EMOJI_BUFF_SIZE];
static char textmsg[300];
#endif

#endif

ProcessMessage TextMessageModule::handleReceived(const meshtastic_MeshPacket &mp)
{
#ifdef FLAMINGO
    /*
     Improve debug messages so that can parsed as part of log at Incident Command
    */
#if defined(DEBUG_PORT) && !defined(DEBUG_MUTE)
    auto rssi = mp.rx_rssi;
    auto &p = mp.decoded;
    meshtastic_NodeInfoLite *n = nodeDB->getMeshNode(getFrom(&mp));

    LOG_INFO("TextModule msg: from=0x%0x, id=0x%x, ln=%s, rxSNR=%g, hop_limit=%d, hop_start=%d", mp.from, mp.id,
             n->user.long_name, mp.rx_snr, mp.hop_limit, mp.hop_start);
    uint16_t offset;
    bool do_loop = 1;
    offset = 0;
    uint16_t bytes_left = handleEmoji((char *)p.payload.bytes, emoji_buf, p.payload.size);
    /* apparently, the maximum size log message is about 150 characters. Deal this with this*/
    while (do_loop) {
        if (bytes_left <= 150) {
            memset(textmsg, 0, bytes_left + 1);
            strncpy(textmsg, (char *)(emoji_buf + offset), bytes_left);
            do_loop = 0;
        } else {
            memset(textmsg, 0, 150 + 1);
            strncpy(textmsg, (char *)(emoji_buf + offset), 150);
            offset = offset + 150;
            bytes_left = bytes_left - 150;
        }
        LOG_INFO("z=%s", textmsg);
    }
    parseAdmin(p.payload.size, (char *)p.payload.bytes, !isBroadcast(mp.to));

#endif
#else
#if defined(DEBUG_PORT) && !defined(DEBUG_MUTE)
    auto &p = mp.decoded;
    LOG_INFO("Received text msg from=0x%0x, id=0x%x, msg=%.*s", mp.from, mp.id, p.payload.size, p.payload.bytes);
#endif
#endif

    // We only store/display messages destined for us.
    // Keep a copy of the most recent text message.
    devicestate.rx_text_message = mp;
    devicestate.has_rx_text_message = true;

    // Only trigger screen wake if configuration allows it
    if (shouldWakeOnReceivedMessage()) {
        powerFSM.trigger(EVENT_RECEIVED_MSG);
    }
    notifyObservers(&mp);

    return ProcessMessage::CONTINUE; // Let others look at this message also if they want
}

bool TextMessageModule::wantPacket(const meshtastic_MeshPacket *p)
{
    return MeshService::isTextPayload(p);
}

/**
 *   Copies inbuf buffer to outbuf buffer and converts
 *   emojis to form :??##@@: where ??,##,@@ are the hex
 *   for the 2nd, 3rd, 4th emoji bytes (21-bit code point)
 *   or the 1st, 2nd, 3rd emoji bytes (16-bit code code)
 *   Since emoji encoding expands the output buffer, there is
 *   chance if payload has many, many emojis that we could
 *   expand past the available space - so the function checks
 *   for this and halts emoji processing if there is not available
 *   space.
 *
 *
 **/

uint16_t TextMessageModule::handleEmoji(char *inbuf, char *outbuf, uint16_t numbytes)
{
    uint16_t i = 0; // index in source buffer
    uint16_t v = 0; // index in dest buffer
    uint32_t emoji_value;
    char tbuf[20];

    // Emoji content transformation will expand the number of bytes
    // in the buffer. We stop processing if we are in dange of overflow

    while (i < numbytes && v < EMOJI_BUFF_SIZE - 8) {
        if ((inbuf[i] & 0xf8) == 0xf0) { // 21-bit Unicode code points, encoded as 4 bytes
            emoji_value = (inbuf[i + 1] << 16) + (inbuf[i + 2] << 8) + inbuf[i + 3];
            sprintf(tbuf, ":%06x:", emoji_value);
            strcpy(outbuf + v, tbuf);
            v = v + 8;                          // :000000:   == 8 characters
            i = i + 4;                          // skip the four emoji bytes
        } else if ((inbuf[i] & 0xe0) == 0xe0) { /// 16-bit Unicode code points, encoded as 3 bytes
            emoji_value = (inbuf[i] << 16) + (inbuf[i + 1] << 8) + inbuf[i + 2];
            sprintf(tbuf, ":%06x:", emoji_value);
            strcpy(outbuf + v, tbuf);
            v = v + 8; // :000000:   == 8 characters
            i = i + 3; // skip the three emjoi bytes
        } else {
            outbuf[v] = inbuf[i];
            v++;
            i++;
        }
    }
    return v; // number of bytes in dest buffer
}
