
/*
SetTime.ino sketch to set the time of DS1307 RTC - created by Paul Stoffregen
github.com/PaulStoffregen/DS1307RTC/blob/master/examples/SetTime/SetTime.ino
 */

#define __AVR_ATtiny85__

#include <Arduino.h> 
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <TimeLib.h>
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

const char *monthName[12] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

tmElements_t tm;
SoftwareSerial serial(1, 4);
// SoftwareSerial serial(0, 0);
volatile uint8_t portbhistory = 0xFF;     // default is high because the pull-up
volatile boolean manualFlag = false;
boolean state = false;

bool getTime(const char *str);
bool getDate(const char *str);

// void isr() {
//   serial.println("ISR");
//   manualFlag = true;
// }

void setup() {
  bool parse=false;
  bool config=false;

  serial.begin(9600);
  while (!serial) ; // wait for Arduino Serial Monitor
  delay(1000);
  Wire.begin();

  pinMode(PIN_MOSFET, OUTPUT);
  pinMode(PIN_BTN_MANUAL, INPUT_PULLUP);
  adc_disable();

  delay(500);
  setSyncProvider(RTC.get);  

  if (timeStatus() != timeSet) {
    serial.println("Unable to sync with the RTC");
  } else {
    serial.println("RTC has set the system time");
  }

  serial.println("Chip status: ");
  serial.println(RTC.chipPresent());

  wdt_reset(); // watchdog init
  // 15MS, 30MS, 60MS, 120MS, 250MS, 500MS, 1S, 2S, 4S, 8S
  wdt_enable(WDTO_4S); // watchdog conf

  // attachInterrupt(digitalPinToInterrupt(PIN_BTN_MANUAL), isr, FALLING);
  WDTCR |= _BV(WDIE);     // enable watchdog interrupt
  GIMSK |= (1<<INT0); // enabling the INT0 (external interrupt) 
  PCMSK |= _BV(PCINT3); // enabling external interrupt on Pin PB2
  sei(); // allow interruptions  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
}

void printDigits(int digits) {
  serial.print(":");
  if (digits < 10)
    serial.print('0');
  serial.print(digits);
}

void loop() {
  static unsigned long periodTimer = 0;
  static unsigned long workTimer = 0;
  static unsigned long debugTimer = 0;
  static unsigned long beforeNextEvent = 0;
  unsigned long currentMillis = millis();

  if (true || currentMillis - debugTimer >= SERIAL_DEBUG_TIME) {
    debugTimer = currentMillis;

    if (!state){
      serial.print("Before next irrigation: ");
      beforeNextEvent = 0.001 * ((periodTimer + PERIOD_TIME) - currentMillis);
      serial.print(String( beforeNextEvent));
    } else {
      serial.print("Before Stop: ");
      beforeNextEvent = 0.001 * ((workTimer + WORK_TIME) - currentMillis);
      serial.print(String( beforeNextEvent ));
    }
    serial.print("s");
    serial.println("");
    
    setSyncProvider(RTC.get);  
    serial.print("Configured Time=");
      serial.print(hour());
    printDigits(minute());
    printDigits(second());
    serial.println();
    // manualFlag = true;
  }

  //serial.println(digitalRead(PIN_BTN_MANUAL) == LOW ? "MNL" : "NO");
  if (manualFlag/*&& !state*/) {
    serial.println("!!!Button click...");
    //serial.println(digitalRead(PIN_BTN_MANUAL) ? "MNL" : "NO");
    // workTimer = currentMillis;
    // periodTimer = currentMillis; do not reset period on manual mode

    state = true;
    pinMode(PIN_MOSFET, OUTPUT);
    digitalWrite(PIN_MOSFET, HIGH);
    manualFlag = false;
  }

  if (!state) {
    if (currentMillis - periodTimer >= PERIOD_TIME) {
      serial.println("Start irrigation...");
      periodTimer = currentMillis;
      workTimer = currentMillis;
      state = true;
      pinMode(PIN_MOSFET, OUTPUT);
      digitalWrite(PIN_MOSFET, HIGH);
    }
  } else {
    if (currentMillis - workTimer >= WORK_TIME) {
      serial.println("Stop irrigation...");
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

bool getTime(const char *str)
{
  int Hour, Min, Sec;

  if (sscanf(str, "%d:%d:%d", &Hour, &Min, &Sec) != 3) return false;
  tm.Hour = Hour;
  tm.Minute = Min;
  tm.Second = Sec;
  return true;
}

bool getDate(const char *str)
{
  char Month[12];
  int Day, Year;
  uint8_t monthIndex;

  if (sscanf(str, "%s %d %d", Month, &Day, &Year) != 3) return false;
  for (monthIndex = 0; monthIndex < 12; monthIndex++) {
    if (strcmp(Month, monthName[monthIndex]) == 0) break;
  }
  if (monthIndex >= 12) return false;
  tm.Day = Day;
  tm.Month = monthIndex + 1;
  tm.Year = CalendarYrToTm(Year);
  return true;
}

ISR (WDT_vect) {
  WDTCR |= _BV(WDIE); // allow interrupt by watchdog
}

ISR (INT0_vect)
{
    uint8_t changedbits;

    changedbits = PINB ^ portbhistory;
    portbhistory = PINB;

    if(changedbits & (1 << PB3))
    {
      serial.println("ISR INT0_vecxt");
      manualFlag = true;
    }
}