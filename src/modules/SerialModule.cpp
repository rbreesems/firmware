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
    SerialModule
        A simple interface to send messages over the mesh network by sending strings
        over a serial port.

        There are no PIN defaults, you have to enable the second serial port yourself.

    Need help with this module? Post your question on the Meshtastic Discourse:
       https://meshtastic.discourse.group

    Basic Usage:

        1) Enable the module by setting enabled to 1.
        2) Set the pins (rxd / rxd) for your preferred RX and TX GPIO pins.
           On tbeam, recommend to use:
                RXD 35
                TXD 15
        3) Set timeout to the amount of time to wait before we consider
           your packet as "done".
        4) not applicable any more
        5) Connect to your device over the serial interface at 38400 8N1.
        6) Send a packet up to 240 bytes in length. This will get relayed over the mesh network.
        7) (Optional) Set echo to 1 and any message you send out will be echoed back
           to your device.

    TODO (in this order):
        * Define a verbose RX mode to report on mesh and packet information.
            - This won't happen any time soon.

    KNOWN PROBLEMS
        * Until the module is initialized by the startup sequence, the TX pin is in a floating
          state. Device connected to that pin may see this as "noise".
        * Will not work on Linux device targets.


*/

#if defined(USE_SLINK)

#define RX_BUFFER 256
#define TIMEOUT 250
#define BAUD 38400
#define ACK 1

// API: Defaulting to the formerly removed phone_timeout_secs value of 15 minutes
#define SERIAL_CONNECTION_TIMEOUT (15 * 60) * 1000UL

SerialModule *serialModule;
SerialModuleRadio *serialModuleRadio;


//SerialModule::SerialModule() : StreamAPI(&Serial2), concurrency::OSThread("Serial") {}
//static Print *serialPrint = &Serial2;
SerialModule::SerialModule() : StreamAPI(&Serial1), concurrency::OSThread("Serial") {}
static Print *serialPrint = &Serial1;

#define MAX_SERIAL_SIZE sizeof(meshtastic_MeshPacket)+8

char serialBytes[MAX_SERIAL_SIZE];
char serialTxBytes[MAX_SERIAL_SIZE];


size_t serialPayloadSize;

SerialModuleRadio::SerialModuleRadio() : MeshModule("SerialModuleRadio")
{
    ourPortNum = meshtastic_PortNum_SERIAL_APP;
    isPromiscuous = true;
    encryptedOk = true;
    
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
    return Throttle::isWithinTimespanMs(lastContactMsec, SERIAL_CONNECTION_TIMEOUT);
}

// define a simple verion of SerialModule that does not have all of the other crap in it
// This is intended for the WisMesh starter kit + RS485 which uses Serial1
// 

int32_t SerialModule::runOnce()
{

    moduleConfig.serial.enabled = true;
    // These pins are RDX0, TXD0 on WisMesh Pocket
    //moduleConfig.serial.rxd = 19;   
    //moduleConfig.serial.txd = 20;
    // These next pins are RDX1, TXD1 on WishMesh  starter kit
    // We would use these if using the WisMesh + RS485 interface
    moduleConfig.serial.rxd = 15;   
    moduleConfig.serial.txd = 16;
    moduleConfig.serial.override_console_serial_port = false;
    moduleConfig.serial.mode = meshtastic_ModuleConfig_SerialConfig_Serial_Mode_DEFAULT;
    moduleConfig.serial.timeout = 1000;
    moduleConfig.serial.echo = 0;
    moduleConfig.serial.baud = meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_38400;

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
            //while (0) {
                serialPayloadSize = Serial1.readBytes(serialBytes, meshtastic_Constants_DATA_PAYLOAD_LEN);
                //serialPayloadSize = Serial1.readBytesUntil(0, serialBytes, MAX_SERIAL_SIZE);
                serialBytes[serialPayloadSize] = 0;
                LOG_INFO("SerialModule Read size: %d,  RX: %s ", serialPayloadSize, serialBytes);
                //serialModuleRadio->sendPayload();
            }
        }
    return (50);
} 


/**
 * Sends telemetry packet over the mesh network.
 *
 * @param m The telemetry data to be sent
 *
 * @return void
 *
 * @throws None
 */
void SerialModule::sendTelemetry(meshtastic_Telemetry m)
{
    meshtastic_MeshPacket *p = router->allocForSending();
    p->decoded.portnum = meshtastic_PortNum_TELEMETRY_APP;
    p->decoded.payload.size =
        pb_encode_to_bytes(p->decoded.payload.bytes, sizeof(p->decoded.payload.bytes), &meshtastic_Telemetry_msg, &m);
    p->to = NODENUM_BROADCAST;
    p->decoded.want_response = false;
    if (config.device.role == meshtastic_Config_DeviceConfig_Role_SENSOR) {
        p->want_ack = true;
        p->priority = meshtastic_MeshPacket_Priority_HIGH;
    } else {
        p->priority = meshtastic_MeshPacket_Priority_RELIABLE;
    }
    service->sendToMesh(p, RX_SRC_LOCAL, true);
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

/**
 * Sends a payload to a specified destination node.
 *
 * @param dest The destination node number.
 * @param wantReplies Whether or not to request replies from the destination node.
 */
void SerialModuleRadio::sendPayload(NodeNum dest, bool wantReplies)
{
    const meshtastic_Channel *ch = (boundChannel != NULL) ? &channels.getByName(boundChannel) : NULL;
    meshtastic_MeshPacket *p = allocReply();
    LOG_INFO("SerialModule: sending payload to node: %x", dest);
    p->to = dest;
    if (ch != NULL) {
        p->channel = ch->index;
    }
    p->decoded.want_response = wantReplies;

    p->want_ack = ACK;

    p->decoded.payload.size = serialPayloadSize; // You must specify how many bytes are in the reply
    memcpy(p->decoded.payload.bytes, serialBytes, p->decoded.payload.size);

    service->sendToMesh(p);
}

bool SerialModuleRadio::wantPacket(const meshtastic_MeshPacket *p) {
    if (p->decoded.portnum == meshtastic_PortNum_TELEMETRY_APP) return false;
    if (p->decoded.portnum == meshtastic_PortNum_POSITION_APP) return false;

    LOG_DEBUG("Serial Module want packet, portnum: %d", p->decoded.portnum);
    return true; 
}

/**
 * Handle a received mesh packet.
 *
 * @param mp The received mesh packet.
 * @return The processed message.
 */
ProcessMessage SerialModuleRadio::handleReceived(const meshtastic_MeshPacket &mp)
{
    if (moduleConfig.serial.enabled) {
        if (moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_PROTO) {
            // in API mode we don't care about stuff from radio.
            return ProcessMessage::CONTINUE;
        }

        auto &p = mp.decoded;
        LOG_DEBUG("Serial Module Received  ourNodeNum:%0x from=0x%0x, to=0x%0x, id=0x%0x, size=%d,  portnum=%d",
                  nodeDB->getNodeNum(), mp.from, mp.to, mp.id, p.payload.size, mp.decoded.portnum);
        if (mp.decoded.portnum == 1) {
            LOG_DEBUG("Serial Module Received msg: %s", p.payload.bytes);
        }
        LOG_DEBUG("Packet max size: %d", MAX_SERIAL_SIZE);
        //sprintf(serialTxBytes, "Hello from RX buffer");
        //serialPrint->printf("hello from TX port");
        //serialPrint->write(p.payload.bytes, p.payload.size);
        //serialPrint->printf("%s", p.payload.bytes);

        // For TX > RX echo test, just write the payload
        if (mp.decoded.portnum == 1) {
            memcpy(serialTxBytes, p.payload.bytes, p.payload.size);
            serialTxBytes[p.payload.size] = 0;
            //sprintf(serialTxBytes, "Hello from RX buffer");
            if (Serial1.availableForWrite()) {
                Serial1.write(serialTxBytes, p.payload.size+1);
            }
        }

    }

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

/**
 * Process the received weather station serial data, extract wind, voltage, and temperature information,
 * calculate averages and send telemetry data over the mesh network.
 *
 * @return void
 */
void SerialModule::processWXSerial()
{

    return;
}

#endif
