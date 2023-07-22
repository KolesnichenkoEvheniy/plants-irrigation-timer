#include <Arduino.h>

#define PERIOD_TIME 60 * 60 * 24 * 3   // period in seconds - 3 days
#define WORK_TIME 20
#define PIN_MOSFET 1

uint32_t mainTimer = 0;
uint32_t myTimer = 0;
boolean state = false;

#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#define adc_disable() (ADCSRA &= ~(1<<ADEN)) // disable ADC (before power-off)
#define adc_enable()  (ADCSRA |=  (1<<ADEN)) // re-enable ADC


void setup() {
  // all pins - input for power saving
  for (byte i = 0; i < 6; i++) {
    pinMode(i, INPUT);
  }
  pinMode(PIN_MOSFET, OUTPUT);
  adc_disable(); // power saving

  wdt_reset(); // watchdog init
  // 15MS, 30MS, 60MS, 120MS, 250MS, 500MS, 1S, 2S, 4S, 8S
  wdt_enable(WDTO_1S); // watchdog conf

  WDTCR |= _BV(WDIE);     // enable watchdog interrupt
  sei(); // allow interruptions
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
}

void loop() {
  mainTimer++;

  if (!state) {
    if ((long)myTimer < 1 || (long)mainTimer - myTimer > PERIOD_TIME) {
      myTimer = mainTimer;
      state = true;
      pinMode(PIN_MOSFET, OUTPUT);
      digitalWrite(PIN_MOSFET, HIGH);
    }
  } else {
    if ((long)mainTimer - myTimer > WORK_TIME) {
      myTimer = mainTimer;
      state = false;
      digitalWrite(PIN_MOSFET, LOW);
      pinMode(PIN_MOSFET, INPUT); // power saving
    }
  }

  sleep_enable(); // allow sleep
  sleep_cpu();// sleep
}

ISR (WDT_vect) {
  WDTCR |= _BV(WDIE); // allow interrupt by watchdog
}
