#include <Arduino.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <SoftwareSerial.h>

#define MILLIS_CORRECT_IS_SUPPURT true
#define __AVR_ATtiny85__
#include <GyverButton.h>
#include <GyverPower.h>

#define PERIOD_TIME 7000 // 60 * 60 * 24 * 3 * 1000   // period in seconds - 3 days
#define WORK_TIME 5000 
#define PIN_MOSFET PB1
#define PIN_BTN_MANUAL PB2

GButton btnManual(PIN_BTN_MANUAL);

boolean state = false;

volatile uint8_t portbhistory = 0xFF;     // default is high because the pull-up
volatile boolean manualFlag = false;

#define adc_disable() (ADCSRA &= ~(1<<ADEN)) // disable ADC (before power-off)
#define adc_enable()  (ADCSRA |=  (1<<ADEN)) // re-enable ADC

SoftwareSerial serial(3, 4);

void isr() {
  // в отличие от sleepDelay, ничего вызывать не нужно!
    // uint8_t changedbits;

    // changedbits = PINB ^ portbhistory;
    // portbhistory = PINB;

    // if(changedbits & (1 << PB2))
    // {
    //   manualFlag = true;
    // }
    serial.println("ISR");
    manualFlag = true;
    power.wakeUp();
}

void setup() {
  // all pins - input for power saving
  for (byte i = 0; i < 6; i++) {
   // pinMode(i, INPUT);
  }
  pinMode(PIN_MOSFET, OUTPUT);
  pinMode(PIN_BTN_MANUAL, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(2), isr, FALLING);

  // глубокий сон
  power.setSleepMode(IDLE_SLEEP);
  adc_disable(); // power saving

  //btnManual.setTickMode(AUTO);

  // wdt_reset(); // watchdog init
  // // 15MS, 30MS, 60MS, 120MS, 250MS, 500MS, 1S, 2S, 4S, 8S
  // wdt_enable(WDTO_4S); // watchdog conf

  // WDTCR |= _BV(WDIE);     // enable watchdog interrupt
  // GIMSK |= (1<<INT0); // enabling the INT0 (external interrupt) 
  // PCMSK |= _BV(PCINT2); // enabling external interrupt on Pin PB2
  // sei(); // allow interruptions  
  // set_sleep_mode(SLEEP_MODE_IDLE);

  serial.begin(9600);
}
 
void loop() {
  static uint32_t periodTimer = 0;
  static uint32_t workTimer = 0;
  static uint32_t tmr;
  unsigned long currentMillis = millis();

  serial.println("Loop.. " + String(millis()));

  btnManual.tick();
  if (btnManual.isClick() || btnManual.isHold()) {
    serial.println("Button click...");
      workTimer = currentMillis;
      periodTimer = currentMillis;

      state = true;
      pinMode(PIN_MOSFET, OUTPUT);
      digitalWrite(PIN_MOSFET, HIGH);
      manualFlag = false;
  }

  if (!state) {
    if (currentMillis - periodTimer >= PERIOD_TIME) {
      serial.println("Period...");
      periodTimer = currentMillis;
      workTimer = currentMillis;
      state = true;
      pinMode(PIN_MOSFET, OUTPUT);
      digitalWrite(PIN_MOSFET, HIGH);
    }
  } else {
    if (currentMillis - workTimer >= WORK_TIME) {
      serial.println("Work...");
      workTimer = currentMillis;
      periodTimer = currentMillis;
      state = false;
      digitalWrite(PIN_MOSFET, LOW);
      pinMode(PIN_MOSFET, INPUT); // power saving
    }
  }

  // sleep_enable(); // allow sleep
  // sleep_cpu();// sleep
  power.sleepDelay(4000);
  //power.sleep(SLEEP_8192MS);

  serial.println("wake up!");
}

// ISR (WDT_vect) {
//   WDTCR |= _BV(WDIE); // allow interrupt by watchdog
// }

// ISR (INT0_vect)
// {
    // uint8_t changedbits;

    // changedbits = PINB ^ portbhistory;
    // portbhistory = PINB;

    // if(changedbits & (1 << PB2))
    // {
    //   manualFlag = true;
    // }
// }