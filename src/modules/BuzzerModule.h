#pragma once

#include "SinglePortModule.h"
#include "concurrency/OSThread.h"
#include "configuration.h"
#include <Arduino.h>
#include <functional>


#define BUZZER_PIN PIN_IO3   // Slot C
//#define BUZZER_PIN PIN_IO1     // Slot A
//#define BUZZER_PIN PIN_IO2     // Slot B
//#define BUZZER_PIN PIN_IO4
#define BUZZER_FULL_TONE 500    //500 ms PWM
#define BUZZER_HALF_TONE 250    //250 ms PWM
#define BUZZER_QUARTER_TONE 125    //125 ms PWM



class BuzzerModule : private concurrency::OSThread
{
    bool firstTime = 1;
    unsigned long toneStarted = 0;   //tone started
    unsigned long toneFinish = 0;   //finish time for tone
    unsigned long pulseStarted = 0;   //pulse started
    unsigned long pulseFinish = 0;   //pulse finish
    unsigned long pauseFinish = 0;
    uint16_t currentTone = 0;     // in MS, will be FULL/HALF/QUARTER  
    uint16_t toneDurationSecs = 0;    // how long one tone should last
    uint16_t tonePauseSecs = 0;       // pause between tones
    uint16_t toneNumber = 0;       // number of tones

  public:
    BuzzerModule();
    void startTone(uint16_t tone, uint16_t duration, uint16_t pause, uint16_t number);

  protected:
    virtual int32_t runOnce() override;
    
    
};

extern BuzzerModule *buzzerModule;

