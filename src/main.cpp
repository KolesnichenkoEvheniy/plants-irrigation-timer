#include <Arduino.h> 
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <DS1307RTC.h>
#include <TinyWireM.h>
#include <SoftwareSerial.h>
#include <EncButton.h>

#define PIN_MOSFET PB1
#define PIN_BTN_MANUAL PB3
#define PIN_UART_TX PB4
#define PIN_UART_RX PB5
#define WORK_TIME 15000

#define Wire TinyWireM

#define adc_disable() (ADCSRA &= ~(1<<ADEN)) // disable ADC (before power-off)
#define adc_enable()  (ADCSRA |=  (1<<ADEN)) // re-enable ADC

const byte schedule[][3] = {
  {dowSaturday, 12, 0},
  {dowWednesday, 12, 0},
};

SoftwareSerial serial(PIN_UART_RX, PIN_UART_TX);
volatile uint8_t portbhistory = 0xFF;     // default is high because the pull-up
volatile boolean manualFlag = false;
static unsigned long workTimer = 0;
boolean currentPumpState = false;

bool getTime(const char *str);
bool getDate(const char *str);
void setPumpStatus(bool status);

EncButton<EB_TICK, PIN_BTN_MANUAL> manualBtn;

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
    serial.println(F("Unable to sync with the RTC"));
  }

  serial.println(F("Chip status: "));
  serial.println(RTC.chipPresent());

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
  manualBtn.tick();
  if (manualBtn.isClick() && !currentPumpState) {
    serial.println(F("!!!Button click..."));
    workTimer = millis();

    setPumpStatus(true);
  }
  manualFlag = false;

  static byte prevMin = 0;
  static byte currentMin = 0;
  currentMin = minute();

  if (!currentPumpState) {
    setSyncProvider(RTC.get);

    serial.println();
    serial.print(F("CURR TIME: ")); serial.print(hour()); serial.print(F(":")); serial.print(minute());
    serial.println();
  }

  if (!currentPumpState && prevMin != currentMin) {
    prevMin = currentMin;
    for (byte i = 0; i < sizeof(schedule) / 3; i++) {
      if (schedule[i][0] == weekday() && schedule[i][1] == hour() && schedule[i][2] == minute()) {
        serial.println(F("Enabling the pump..."));
        workTimer = millis();
        setPumpStatus(true);
      }
    }
  }

  if (currentPumpState && (millis() - workTimer >= WORK_TIME)) {
    workTimer = millis();
    setPumpStatus(false);
  }

  if (!currentPumpState && !manualFlag) {
    sleep_enable(); // allow sleep
    sleep_cpu();// sleep
  }
}

void setPumpStatus(bool newState) {
  serial.println(F("Change status: ")); serial.print(newState);
  currentPumpState = newState;
  if (newState == true) {
    pinMode(PIN_MOSFET, OUTPUT);
    digitalWrite(PIN_MOSFET, HIGH);
    return;
  }
  digitalWrite(PIN_MOSFET, LOW);
  pinMode(PIN_MOSFET, INPUT); // power saving
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
      manualBtn.tickISR();
    }
}