#include <Arduino.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#define PERIOD_TIME 259200000 // 60 * 60 * 24 * 3 * 1000   // period in seconds - 3 days
#define WORK_TIME 25000 
#define PIN_MOSFET PB1
#define PIN_BTN_MANUAL PB2

boolean state = false;

volatile uint8_t portbhistory = 0xFF;     // default is high because the pull-up
volatile boolean manualFlag = false;

#define adc_disable() (ADCSRA &= ~(1<<ADEN)) // disable ADC (before power-off)
#define adc_enable()  (ADCSRA |=  (1<<ADEN)) // re-enable ADC

void setup() {
  // all pins - input for power saving
  for (byte i = 0; i < 6; i++) {
    pinMode(i, INPUT);
  }
  pinMode(PIN_MOSFET, OUTPUT);
  pinMode(PIN_BTN_MANUAL, INPUT_PULLUP);
  adc_disable(); // power saving

  wdt_reset(); // watchdog init
  // 15MS, 30MS, 60MS, 120MS, 250MS, 500MS, 1S, 2S, 4S, 8S
  wdt_enable(WDTO_4S); // watchdog conf

  WDTCR |= _BV(WDIE);     // enable watchdog interrupt
  GIMSK |= (1<<INT0); // enabling the INT0 (external interrupt) 
  PCMSK |= _BV(PCINT2); // enabling external interrupt on Pin PB2
  sei(); // allow interruptions  
  set_sleep_mode(SLEEP_MODE_IDLE);
}
 
void loop() {
  static uint32_t periodTimer = 0;
  static uint32_t workTimer = 0;
  unsigned long currentMillis = millis();

  if (manualFlag) {
      workTimer = currentMillis;
      periodTimer = currentMillis;

      state = true;
      pinMode(PIN_MOSFET, OUTPUT);
      digitalWrite(PIN_MOSFET, HIGH);
      manualFlag = false;
  }

  if (!state) {
    if (currentMillis - periodTimer >= PERIOD_TIME) {
      periodTimer = currentMillis;
      workTimer = currentMillis;
      state = true;
      pinMode(PIN_MOSFET, OUTPUT);
      digitalWrite(PIN_MOSFET, HIGH);
    }
  } else {
    if (currentMillis - workTimer >= WORK_TIME) {
      workTimer = currentMillis;
      periodTimer = currentMillis;
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

ISR (INT0_vect)
{
    uint8_t changedbits;

    changedbits = PINB ^ portbhistory;
    portbhistory = PINB;

    if(changedbits & (1 << PB2))
    {
      manualFlag = true;
    }
}