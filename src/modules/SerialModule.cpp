#include "SerialModule.h"
#include "GeoCoord.h"
#include "MeshService.h"
#include "NMEAWPL.h"
#include "NodeDB.h"
#include "RTC.h"
#include "Router.h"
#include "configuration.h"
#include <Arduino.h>
#include <Throttle.h>

/*

    This module has been totally rewritten to function as a serial 'interface'
    for the router to send packets over the serial link similar to the MQTT interface.
  
    The serial link is simply an alternate path for packets other than the air.

    This is not a module, it does not source new packets or sink packets

    This is intended for the WisMesh starter kit (19007 board+ 4630) + RS485 which uses Serial1

*/

#if defined(USE_SLINK)

#define TIMEOUT 250
#define BAUD 38400
#define ACK 1

// API: Defaulting to the formerly removed phone_timeout_secs value of 15 minutes
#define SERIAL_CONNECTION_TIMEOUT (15 * 60) * 1000UL

#define PACKET_FLAGS_ENCRYPTED_MASK PACKET_FLAGS_VIA_MQTT_MASK

SerialModule *serialModule;
SerialModuleRadio *serialModuleRadio;

meshtastic_serialPacket outPacket;
meshtastic_serialPacket inPacket;
char tmpbuf[250];  // for debug only



SerialModule::SerialModule() : StreamAPI(&Serial1), concurrency::OSThread("Serial") {}
static Print *serialPrint = &Serial1;

#define headerByte1 0xaa
#define headerByte2 0x55


size_t serialPayloadSize;

uint32_t computeCrc32(const uint8_t* buf, uint16_t len) {
  uint32_t crc = 0xFFFFFFFF; // Initial value
  const uint32_t poly = 0xEDB88320; // CRC-32 polynomial

  for (uint16_t i = 0; i < len; i++) {
    crc ^= (uint8_t)buf[i]; // XOR with the current byte
    for (int j = 7; j >= 0; j--) { // Perform 8 bitwise operations
      if (crc & 0x80000000) { // Check if the MSB is set
        crc = (crc << 1) ^ poly; // Shift and XOR with polynomial
      } else {
        crc <<= 1; // Shift if MSB is not set
      }
    }
  }
  return ~crc; // Return the final CRC value
}


void meshPacketToSerialPacket (const meshtastic_MeshPacket &mp, meshtastic_serialPacket *sp) {
    sp->header.hbyte1 = headerByte1;
    sp->header.hbyte2 = headerByte2;
    sp->header.crc = 0;
    
    if (mp.which_payload_variant == meshtastic_MeshPacket_encrypted_tag ){
        sp->header.size = sizeof(SerialPacketHeader) + mp.encrypted.size;
        memcpy(sp->payload, mp.encrypted.bytes, mp.encrypted.size);
    } else {
        sp->header.size = sizeof(SerialPacketHeader) + mp.decoded.payload.size;
        memcpy(sp->payload, mp.decoded.payload.bytes, mp.decoded.payload.size);
    }
    sp->header.from = mp.from;
    sp->header.to = mp.to;
    sp->header.id = mp.id;
    sp->header.channel = mp.channel;
    
    sp->header.hop_limit = mp.hop_limit & PACKET_FLAGS_HOP_LIMIT_MASK;
    sp->header.hop_start = mp.hop_start & PACKET_FLAGS_HOP_START_MASK;
    sp->header.flags =
        0x20 | (mp.want_ack ? PACKET_FLAGS_WANT_ACK_MASK : 0) | ((mp.which_payload_variant == meshtastic_MeshPacket_encrypted_tag) ? PACKET_FLAGS_ENCRYPTED_MASK : 0);

    sp->header.crc = computeCrc32((const uint8_t *)sp, sp->header.size);
}

void insertSerialPacketToMesh(meshtastic_serialPacket *sp) {

    UniquePacketPoolPacket p = packetPool.allocUniqueZeroed();

    p->from = sp->header.from;
    p->to = sp->header.to;
    p->id = sp->header.id;
    p->channel = sp->header.channel;
    //assert(HOP_MAX <= PACKET_FLAGS_HOP_LIMIT_MASK); // If hopmax changes, carefully check this code
    p->hop_limit = sp->header.hop_limit;
    p->hop_start = sp->header.hop_start;
    p->want_ack = !!(sp->header.flags & PACKET_FLAGS_WANT_ACK_MASK);
    p->via_slink = true;
    p->via_mqtt = 0;
    uint16_t payloadLen = sp->header.size - sizeof(SerialPacketHeader);
    if (!!(sp->header.flags & PACKET_FLAGS_ENCRYPTED_MASK)) {
        p->which_payload_variant = meshtastic_MeshPacket_encrypted_tag;
        memcpy(p->encrypted.bytes, sp->payload, payloadLen);
        p->encrypted.size = payloadLen;
    } else {
        p->which_payload_variant = meshtastic_MeshPacket_decoded_tag;
        memcpy(p->decoded.payload.bytes, sp->payload,  payloadLen);
        p->decoded.payload.size = payloadLen;
    }

    LOG_DEBUG ("Serial Module RX  from=0x%0x, to=0x%0x, packet_id=0x%0x",
              p->from, p->to, p->id);

    if (p->which_payload_variant == meshtastic_MeshPacket_decoded_tag) {
        memcpy(tmpbuf, p->decoded.payload.bytes, p->decoded.payload.size);
        tmpbuf[p->decoded.payload.size+1]=0;
        LOG_DEBUG("Serial Module RX packet of %d bytes, msg: %s", sp->header.size, tmpbuf);
    }
                    
    router->enqueueReceivedMessage(p.release());

}




// check if this recieved serial packet is valid
bool checkIfValidPacket(meshtastic_serialPacket *sp) {

    if (sp->header.hbyte1 != headerByte1 || sp->header.hbyte2 != headerByte2 ) {
        LOG_DEBUG("SerialModule:: valid packet check fail, header bytes");
        return false;
    }
    if (sp->header.size == 0 || sp->header.size > sizeof(meshtastic_serialPacket)) {
        LOG_DEBUG("SerialModule:: valid packet check fail, invalid size");
        return false;
    }
    
    uint32_t received_crc = sp->header.crc;
    sp->header.crc = 0; // need to set to zero for computing CRC
    if (computeCrc32((const uint8_t *)sp, sp->header.size) != received_crc) {
        LOG_DEBUG("SerialModule:: valid packet check fail, invalid crc");
        sp->header.crc = received_crc; // restore
        return false;
    }
    sp->header.crc = received_crc; // restore
    return true;
}

SerialModuleRadio::SerialModuleRadio() : MeshModule("SerialModuleRadio")
{
    ourPortNum = meshtastic_PortNum_SERIAL_APP;
    
}


// define a simple verion of SerialModule that does not have all of the other crap in it
// This is intended for the WisMesh starter kit + RS485 which uses Serial1
// 

int32_t SerialModule::runOnce()
{

    moduleConfig.serial.enabled = true;
    // These pins are RDX0, TXD0 on WisMesh Pocket. Cannot use this in production
    // as the WisMesh pocket has a GPS on UART1, which clashes with the RS485 board
    //moduleConfig.serial.rxd = 19;   
    //moduleConfig.serial.txd = 20;
    // These next pins are RDX1, TXD1 on WishMesh  starter kit
    // We would use these if using the WisMesh + RS485 interface
    moduleConfig.serial.rxd = 15;   
    moduleConfig.serial.txd = 16;
    moduleConfig.serial.override_console_serial_port = false;
    moduleConfig.serial.mode = meshtastic_ModuleConfig_SerialConfig_Serial_Mode_DEFAULT;
    moduleConfig.serial.timeout = TIMEOUT;
    moduleConfig.serial.echo = 0;
    moduleConfig.serial.baud = meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_19200;

    if (!moduleConfig.serial.enabled)
        return disable();

    if (firstTime) {
        // Interface with the serial peripheral from in here.
        LOG_INFO("Init serial peripheral interface");

        uint32_t baud = getBaudRate();
        Serial1.setPins(moduleConfig.serial.rxd, moduleConfig.serial.txd);
        Serial1.begin(baud, SERIAL_8N1);
        Serial1.setTimeout(moduleConfig.serial.timeout > 0 ? moduleConfig.serial.timeout : TIMEOUT);
        serialModuleRadio = new SerialModuleRadio();
        firstTime = 0;
    } else {
            //stream.cpp/readBytes  arduinofruit library
            while (Serial1.available()) {
                serialPayloadSize = Serial1.readBytes((uint8_t *) &inPacket, sizeof(meshtastic_serialPacket));
                 if (!checkIfValidPacket(&inPacket)) {
                    LOG_DEBUG("Serial Module failed CRC on RX");
                } else {
                    // checks passed, pass this packet on
                    LOG_DEBUG("Serial Module RX Insert packet to mesh");
                    insertSerialPacketToMesh(&inPacket);
                }
            }
        }
    return (50);
} 

/**
 * @brief Checks if the serial connection is established.
 *
 * @return true if the serial connection is established, false otherwise.
 *
 * For the serial2 port we can't really detect if any client is on the other side, so instead just look for recent messages
 */
bool SerialModule::checkIsConnected()
{
    //return Throttle::isWithinTimespanMs(lastContactMsec, SERIAL_CONNECTION_TIMEOUT);
    // we are not going to be able to determine if connected to another radio or not
    // just always return true
    // not sure where this function is called
    return true;  
}


/**
 * Allocates a new mesh packet for use as a reply to a received packet.
 *
 * @return A pointer to the newly allocated mesh packet.
 */
meshtastic_MeshPacket *SerialModuleRadio::allocReply()
{
    auto reply = allocDataPacket(); // Allocate a packet for sending

    return reply;
}

bool SerialModuleRadio::wantPacket(const meshtastic_MeshPacket *p) {
    // never accept packets from module handler as we are relying on sampling the RX input
    return false;
}

/*
 Called from Router.cpp/Router::send
 Send this over the link
*/
void SerialModuleRadio::onSend(const meshtastic_MeshPacket &mp) {

    if (mp.via_slink) {
        LOG_DEBUG("Serial Module Onsend TX - ignoring packet that came from slink");
    }
    
    LOG_DEBUG("Serial Module Onsend TX   from=0x%0x, to=0x%0x, packet_id=0x%0x",
              mp.from, mp.to, mp.id);
    meshPacketToSerialPacket(mp, &outPacket);
    // debug check
    if (!checkIfValidPacket(&outPacket)) {
        LOG_DEBUG("Serial Module failed CRC on TX");
    } else {
        if (Serial1.availableForWrite()) {
            LOG_DEBUG("Serial Module onSend TX packet of %d bytes", outPacket.header.size);
            Serial1.write((uint8_t *) &outPacket, outPacket.header.size);
        }
    }

}

/**
 * Handle a received mesh packet.
 *
 * @param mp The received mesh packet.
 * @return The processed message.
 */
ProcessMessage SerialModuleRadio::handleReceived(const meshtastic_MeshPacket &mp)
{
    // we are never going to handle packets when called from the Module handler
    return ProcessMessage::CONTINUE; // Let others look at this message also if they want
}

/**
 * @brief Returns the baud rate of the serial module from the module configuration.
 *
 * @return uint32_t The baud rate of the serial module.
 */
uint32_t SerialModule::getBaudRate()
{
    if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_110) {
        return 110;
    } else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_300) {
        return 300;
    } else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_600) {
        return 600;
    } else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_1200) {
        return 1200;
    } else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_2400) {
        return 2400;
    } else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_4800) {
        return 4800;
    } else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_9600) {
        return 9600;
    } else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_19200) {
        return 19200;
    } else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_38400) {
        return 38400;
    } else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_57600) {
        return 57600;
    } else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_115200) {
        return 115200;
    } else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_230400) {
        return 230400;
    } else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_460800) {
        return 460800;
    } else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_576000) {
        return 576000;
    } else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_921600) {
        return 921600;
    }
    return BAUD;
}


#endif
