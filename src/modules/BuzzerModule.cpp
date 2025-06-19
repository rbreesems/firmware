/**
 * @file BuzzerModule.cpp
 * @brief Implements a PWM Buzzer
 *
 */

#include "Default.h"
#include "BuzzerModule.h"
#include "FSCommon.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "PowerFSM.h"
#include "RTC.h"
#include "Router.h"
#include "SPILock.h"
#include "airtime.h"
#include "configuration.h"
#include "gps/GeoCoord.h"
#include <Arduino.h>
#include <Throttle.h>

BuzzerModule *buzzerModule;

BuzzerModule::BuzzerModule() : concurrency::OSThread("Buzzer") {}

// runOnce is really misnamed - this is periodically called

bool initDone = 0;


#define POLL_INTERVAL 25   // 25 ms

#define STATE_DEFAULT 0
#define STATE_HIGH 1
#define STATE_LOW 2
#define STATE_PAUSE 3

uint8_t fsmState = STATE_DEFAULT;

int32_t BuzzerModule::runOnce()
{
    if (!initDone) {
        pinMode(BUZZER_PIN, OUTPUT);
        digitalWrite(BUZZER_PIN, LOW);
        initDone = 1;
    }

    unsigned long now = millis();

    /*
    A tone is a sequence of PWM pulses. 
    Tone duration sets the number of PWM pulses that will be done.
    Tone type (FULL/HALF/QUARTER) sets the high pulse width of the PWM
    Tone number is the number of tones that will be done
    Tone pause is the pause in seconds between tones
    */

    if (currentTone != 0) {
        pinMode(BUZZER_PIN, OUTPUT);
        LOG_INFO("Buzzer, starting tone");
        unsigned long stop = now + 3000;
        while (millis() < stop) {
            digitalWrite(BUZZER_PIN, HIGH);
            delayMicroseconds(1200);
            digitalWrite(BUZZER_PIN, LOW);
            delayMicroseconds(1200);
        }
        currentTone = 0;
        LOG_INFO("Buzzer, stopped tone");
    }

#if 0

    switch(fsmState) {

        case STATE_DEFAULT:
            if (currentTone != 0) {
                // start a new tone
                LOG_INFO("Buzzer, starting tone");
                toneStarted = now;
                pulseStarted = now;
                toneFinish = now + toneDurationSecs*1000;
                pulseFinish = now + currentTone;
                digitalWrite(BUZZER_PIN, HIGH);
                fsmState = STATE_HIGH;
            } else {
                digitalWrite(BUZZER_PIN, LOW);
            }
            break;
        case STATE_HIGH:
            if (now > pulseFinish){
                digitalWrite(BUZZER_PIN, LOW);
                fsmState = STATE_LOW;
            }
            break;
        case STATE_LOW:
            // all cycles are 1 second
            if  (now > (pulseStarted + 1000)) {
                // one cycle is has finished
                if (now < toneFinish) {
                    //start another pulse
                    pulseFinish = now + currentTone;
                    digitalWrite(BUZZER_PIN, HIGH);
                    fsmState = STATE_HIGH;
                } else {
                    // a tone has finished
                    fsmState = STATE_DEFAULT;  
                    if (toneNumber != 0) toneNumber--;
                    if (toneNumber == 0) {
                        currentTone = 0;  //clear current Tone
                        LOG_INFO("Buzzer, finished tone");
                    } else if (tonePauseSecs != 0) {
                        pauseFinish = now + tonePauseSecs*1000;
                        fsmState = STATE_PAUSE;  
                    }
                    
                }
            }
            break;
        case STATE_PAUSE:
            if  (now > pauseFinish ) {
                fsmState = STATE_DEFAULT; 
            }
            break;
    }

#endif
    return(POLL_INTERVAL);
}

void BuzzerModule:: startTone(uint16_t tone, uint16_t duration, uint16_t pause, uint16_t number) {
    if (fsmState == STATE_DEFAULT && toneNumber == 0) {
        currentTone = tone;
        toneDurationSecs = duration;
        tonePauseSecs = pause;
        toneNumber = number;
        LOG_INFO("Buzzer, starting tone type: %d , duration(secs): %d, pause(secs): %d,  number: %d", tone, duration, pause, number);
    }
}


