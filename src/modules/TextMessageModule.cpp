#include "TextMessageModule.h"
#include "MeshService.h"
#include "MeshTypes.h"
#include "NodeDB.h"
#include "PowerFSM.h"
#include "buzz.h"
#include "configuration.h"
#include "RangeTestModule.h"

TextMessageModule *textMessageModule;

#define MAX_ADMIN_MSG 31

void parseAdmin(pb_size_t size, char* payload){
    char local_payload[MAX_ADMIN_MSG+1];
    pb_size_t new_size;
    if (size < 4) return;  // too short to be an ADMIN message
    if (!( (payload[0] == 'A' || payload[0] == 'a') && (payload[1] == 'D' || payload[1] == 'd'))) return;
    // this is an ADMIN message
    new_size = (size < MAX_ADMIN_MSG) ? size : MAX_ADMIN_MSG;
    strncpy(local_payload, payload, new_size);
    local_payload[new_size] = '\0';
    for (int i = 0; i < new_size; i++) {
        local_payload[i] = tolower(local_payload[i]);
    }
    if (strcmp("adrt on hop", local_payload) == 0) {
        LOG_INFO("Turning Dynamic Rangetest ON with hop");
        setRtDynamicEnable(1);
        setRtHop(1);
    }
    else if (strcmp("adrt on", local_payload) == 0) {
        LOG_INFO("Turning Dynamic Rangetest ON");
        setRtDynamicEnable(1);
        setRtHop(0);
    }
    else if (strcmp("adrt off", local_payload) == 0) {
        LOG_INFO("Turning Dynamic Rangetest OFF");
        setRtDynamicEnable(0);
    }
    else if (strncmp("adrt delay", local_payload, 10) == 0 && new_size >= 13) {
        if (strncmp("15", local_payload+11, 2) == 0) {
            LOG_INFO("Rangetest delay is 15");
            moduleConfig.range_test.sender = 15;
        } 
        else if (strncmp("30", local_payload+11, 2) == 0) {
            LOG_INFO("Rangetest delay is 30");
            moduleConfig.range_test.sender = 30;
        }
        else if (strncmp("60", local_payload+11, 2) == 0) {
            LOG_INFO("Rangetest delay is 60");
            moduleConfig.range_test.sender = 60;
        } 
    }

}


ProcessMessage TextMessageModule::handleReceived(const meshtastic_MeshPacket &mp)
{
    
#ifdef DEBUG_PORT
    auto rssi = mp.rx_rssi;
    auto &p = mp.decoded;
    meshtastic_NodeInfoLite *n = nodeDB->getMeshNode(getFrom(&mp));
    LOG_INFO("TextModule msg: from=0x%0x, id=0x%x, ln=%s, rxSNR=%g, hop_limit=%d, hop_start=%d, msg=%.*s",
        mp.from, mp.id, n->user.long_name, mp.rx_snr, mp.hop_limit, mp.hop_start, p.payload.size, p.payload.bytes);
    if (!isBroadcast(mp.to)) {
        // Direct message, check if admin
        parseAdmin(p.payload.size, (char *)p.payload.bytes);
    }
    

#endif
    // We only store/display messages destined for us.
    // Keep a copy of the most recent text message.
    devicestate.rx_text_message = mp;
    devicestate.has_rx_text_message = true;

    powerFSM.trigger(EVENT_RECEIVED_MSG);
    notifyObservers(&mp);

    return ProcessMessage::CONTINUE; // Let others look at this message also if they want
}

bool TextMessageModule::wantPacket(const meshtastic_MeshPacket *p)
{
    return MeshService::isTextPayload(p);
}