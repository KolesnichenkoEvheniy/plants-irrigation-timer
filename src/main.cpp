
/*
SetTime.ino sketch to set the time of DS1307 RTC - created by Paul Stoffregen
github.com/PaulStoffregen/DS1307RTC/blob/master/examples/SetTime/SetTime.ino
 */

#define __AVR_ATtiny85__

#include <Arduino.h> 
// #include <Wire.h>
#include <TimeLib.h>
#include <DS1307RTC.h>
#include <TinyWireM.h>
#include <SoftwareSerial.h>

#define Wire TinyWireM

const char *monthName[12] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

tmElements_t tm;
SoftwareSerial serial(3, 4);

bool getTime(const char *str);
bool getDate(const char *str);



void setup() {
  bool parse=false;
  bool config=false;

  serial.begin(9600);
  while (!serial) ; // wait for Arduino Serial Monitor
  delay(3000);
  Wire.begin();

  // serial.println("Initial");
  // serial.println(__DATE__);
  // serial.println(tm.Hour);
  // serial.println(tm.Minute);
  // setTime(8,29,0,1,1,11);
  // // get the date and time the compiler was run
  if (getDate(__DATE__) && getTime(__TIME__)) {
    parse = true;
    // and configure the RTC with this info
    // serial.println("before");
  
    // Wire.beginTransmission(0x68);
    // Wire.end();
    // serial.println("after");
    // if (RTC.write(tm)) {
    //   config = true;
    //   serial.println("YES");
    // } else {
    //   serial.println("still no");
    // }
    

      delay(500);
      setSyncProvider(RTC.get);  

     // time_t time = RTC.get();
    if (timeStatus() != timeSet) 
      serial.println("Unable to sync with the RTC");
    else
      serial.println("RTC has set the system time");

      serial.println("Chip status: ");
    serial.println(RTC.chipPresent());
    
   
  }


  
  serial.print("TEST d: ");
  serial.println(tm.Year);
  // if (parse && config) {
  //   serial.print("DS1307 configured Time=");
  //   serial.print(__TIME__);
  //   serial.print(", Date=");
  //   serial.println(__DATE__);
  // } else if (parse) {
  //   serial.println("DS1307 Communication Error :-{");
  //   serial.println("Please check your circuitry");
  // } else {
  //   serial.print("Could not parse info from the compiler, Time=\"");
  //   serial.print(__TIME__);
  //   serial.print("\", Date=\"");
  //   serial.print(__DATE__);
  //   serial.println("\"");
  // }
}

void printDigits(int digits) {
  serial.print(":");
  if (digits < 10)
    serial.print('0');
  serial.print(digits);
}

void loop() {
  delay(1000);
  serial.print("Configured Time=");
    serial.print(hour());
  printDigits(minute());
  printDigits(second());
  serial.println();
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
