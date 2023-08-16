#include <Arduino.h> 
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <DS1307RTC.h>
#include <TinyWireM.h>
#include <SoftwareSerial.h>

#define PIN_MOSFET PB1
#define PIN_BTN_MANUAL PB3
#define PERIOD_TIME 7000//60 * 60 * 24 * 3 * 1000   // period in seconds - 3 days
#define WORK_TIME 3000
#define SERIAL_DEBUG_TIME 2000

#define Wire TinyWireM

#define adc_disable() (ADCSRA &= ~(1<<ADEN)) // disable ADC (before power-off)
#define adc_enable()  (ADCSRA |=  (1<<ADEN)) // re-enable ADC

const byte schedule[][3] = {
  {dowSaturday, 7, 0},
  {dowWednesday, 12, 0},
};

SoftwareSerial serial(1, 4);
volatile uint8_t portbhistory = 0xFF;     // default is high because the pull-up
volatile boolean manualFlag = false;
static unsigned long workTimer = 0;
boolean state = false;

bool getTime(const char *str);
bool getDate(const char *str);

void setup() {
  serial.begin(9600);
  while (!serial) ; // wait for Arduino Serial Monitor
  delay(200);
  Wire.begin();

  pinMode(PIN_MOSFET, OUTPUT);
  pinMode(PIN_BTN_MANUAL, INPUT_PULLUP);
  adc_disable();

  delay(500);
  setSyncProvider(RTC.get);  

  if (timeStatus() != timeSet) {
    serial.println("Unable to sync with the RTC");
  }

  // serial.println("Chip status: ");
  // serial.println(RTC.chipPresent());

  wdt_reset(); // watchdog init
  // 15MS, 30MS, 60MS, 120MS, 250MS, 500MS, 1S, 2S, 4S, 8S
  wdt_enable(WDTO_8S); // watchdog conf

  WDTCR |= _BV(WDIE);     // enable watchdog interrupt
  GIMSK |= (1<<INT0); // enabling the INT0 (external interrupt) 
  PCMSK |= _BV(PCINT3); // enabling external interrupt on Pin PB2
  sei(); // allow interruptions  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
}

void loop() {
  static tmElements_t now;
 
  if (manualFlag && !state) {
    serial.println("!!!Button click...");
    workTimer = millis();

    state = true;
    pinMode(PIN_MOSFET, OUTPUT);
    digitalWrite(PIN_MOSFET, HIGH);
    manualFlag = false;
  }

  static byte prevMin = 0;

  if (!state) {
    if (!RTC.read(now)) {
      serial.println("ERR RTC READ");
    }
  }

  serial.print("CURR TIME: "); serial.print(now.Hour); serial.print(" : "); serial.print(now.Minute);
  if (!state && prevMin != now.Minute) {
    prevMin = now.Minute;
    for (byte i = 0; i < sizeof(schedule) / 3; i++) {
      if (schedule[i][0] == now.Wday && schedule[i][1] == now.Hour && schedule[i][2] == now.Minute) {
        state = true;
        workTimer = millis();
        pinMode(PIN_MOSFET, OUTPUT);
        digitalWrite(PIN_MOSFET, HIGH);
      }
    }
  }

  if (state && millis() - workTimer >= WORK_TIME) {
    workTimer = millis();
    state = false;
    digitalWrite(PIN_MOSFET, LOW);
    pinMode(PIN_MOSFET, INPUT); // power saving
  }

  if (millis() - workTimer >= WORK_TIME) {
    workTimer = millis();
    state = false;
    digitalWrite(PIN_MOSFET, LOW);
    pinMode(PIN_MOSFET, INPUT); // power saving
  }

  if (!state) {
    sleep_enable(); // allow sleep
    sleep_cpu();// sleep
  }
}

ISR (WDT_vect) {
  WDTCR |= _BV(WDIE); // allow interrupt by watchdog
}

ISR (INT0_vect)
{
    uint8_t changedbits;

    changedbits = PINB ^ portbhistory;
    portbhistory = PINB;

    if(changedbits & (1 << PB3)) {
      manualFlag = true;
    }
}