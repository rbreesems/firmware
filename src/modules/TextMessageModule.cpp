#include "TextMessageModule.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "PowerFSM.h"
#include "buzz.h"
#include "configuration.h"
TextMessageModule *textMessageModule;

ProcessMessage TextMessageModule::handleReceived(const meshtastic_MeshPacket &mp)
{
#ifdef DEBUG_PORT
    auto &p = mp.decoded;
    meshtastic_NodeInfoLite *n = nodeDB->getMeshNode(getFrom(&mp));
    LOG_INFO("TextModule msg: from=0x%0x, id=0x%x, ln=%s, rxSNR=%g, hop_limit=%d, hop_start=%d, msg=%.*s",
        mp.from, mp.id, n->user.long_name, mp.rx_snr, mp.hop_limit, mp.hop_start, p.payload.size, p.payload.bytes);
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