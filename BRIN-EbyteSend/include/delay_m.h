#ifndef DELAY_H
#define DELAY_H
#ifndef Arduino_h
#include <Arduino.h>
#endif
// class
uint32_t prevMillis = 0;
uint32_t prevMicros = 0;

void DelayMillis(uint32_t _delay)
{
    uint32_t currentMillis = millis();
    prevMillis = currentMillis;
    while (millis() - prevMillis < _delay){}
}

void DelayMicros(uint32_t _delay)
{
    uint32_t currentMillis = micros();
    prevMillis = currentMillis;
    while (micros() - prevMillis < _delay){}
}

#endif