// ESP8266 + ATtiny84A + DHT22
// version 0.03 - 28.09.2015
// Copyright (c) 2015, meigrafd
// Released under the MIT license.
//------------------------------------------------------------------------------

#include <avr/sleep.h>
#include <avr/wdt.h>  // http://www.nongnu.org/avr-libc/user-manual/group__avr__watchdog.html
#include <SoftwareSerial.h>

#include <DHT22.h>  // https://github.com/nathanchantrell/Arduino-DHT22

// Power-Save-Stuff.
// http://www.surprisingedge.com/low-power-atmegatiny-with-watchdog-timer/
// https://www.sparkfun.com/tutorials/309
// http://jeelabs.org/tag/lowpower/
uint8_t watchdog_counter;
// Watchdog Interrupt Service / is executed when  watchdog timed out
ISR(WDT_vect) { watchdog_counter++; }

//------------------------------------------------------------------------------
// 0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms, 5=500ms
// 6=1sec, 7=2sec, 8=4sec, 9=8sec
// From: http://interface.khm.de/index.php/lab/experiments/sleep_watchdog_battery/
#define watchdog_wakeup  9  // Wake up after 8 sec

// wait this many ms between sending packets. 300000ms / 1000 / 60sec = 5 Min.
#define SENDDELAY  300000
//#define SENDDELAY  10000 //for tests only: 10sec

// 7,5 * 8sec = 1 Min. 37,5 * 8sec = 5Min.
//uint8_t watchdog_limit = SENDDELAY / 1000 / 8;
#define watchdog_limit  SENDDELAY / 1000 / 8

//------------------------------------------------------------------------------

//WiFi AP information. comment out to use settings from config.lua
//#define SSID "bla"
//#define PASS "secret"

// Supply Voltage Limit - send Notify (mV)
#define vccNotice 3100

// Supply Voltage Limit - Shutdown TinySensor and deny operation (mV)
#define vccShutdown 3000

// ESP8266 SoftwareSerial BAUDRATE. Make sure ESP8266 is set to this
#define ESP8266_BAUD 9600

//Comment to turn off Debug mode
#define DEBUG
#define DEBUGBAUD 9600

//------------------------------------------------------------------------------
// PIN-Konfiguration 
//------------------------------------------------------------------------------
//Pin definition for Attiny84 and regular arduino
#if defined(__AVR_ATtiny84__)
  #define ESP_ENABLE_PIN  2
  #define SOFT_RX_PIN     1
  #define SOFT_TX_PIN     0
  #define DHT22_PIN       3
  #define DHT22_POWER_PIN 4
#else
  //ATMEGA328
  #define ESP_ENABLE_PIN  4
  #define SOFT_RX_PIN     2
  #define SOFT_TX_PIN     3
  #define DHT22_PIN       5
  #define DHT22_POWER_PIN 6
#endif
/*
                     +-\/-+
               VCC  1|    |14  GND
          (D0) PB0  2|    |13  AREF (D10)
          (D1) PB1  3|    |12  PA1 (D9)
       (PB3) RESET  4|    |11  PA2 (D8)
INT0  PWM (D2) PB2  5|    |10  PA3 (D7)
      PWM (D3) PA7  6|    |9   PA4 (D6) SCK
SDA   PWM (D4) PA6  7|    |8   PA5 (D5) PWM
                     +----+
*/
//------------------------------------------------------------------------------

// Buffer to store incoming commands from serial port
String inData;

//Remove debug if using attiny84
#if defined(__AVR_ATtiny84__)
  #undef DEBUG
#endif

#ifdef DEBUG
  #define DEBUG_CONNECT(x)  Serial.begin(x)
  #define DEBUG_PRINT(x)    Serial.print(x)
  #define DEBUG_PRINTLN(x)  Serial.println(x)
  #define DEBUG_FLUSH()     Serial.flush()
#else
  #define DEBUG_CONNECT(x)
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_FLUSH()
#endif

//How many times to retry talking to module before giving up
#define RETRY_COUNT   2
uint8_t retry_attempt = 0;

// Setup a DHT22 instance
DHT22 myDHT22(DHT22_PIN);

//Software serial connected to ESP8266
SoftwareSerial espSerial(SOFT_RX_PIN, SOFT_TX_PIN); // RX, TX

//------------------------------------------------------------------------------
// Power Save Functions
//------------------------------------------------------------------------------

// send ATtiny into Power Save Mode
void goToSleep() {
  // SLEEP_MODE_IDLE -the least power savings
  // SLEEP_MODE_ADC
  // SLEEP_MODE_PWR_SAVE
  // SLEEP_MODE_STANDBY
  // SLEEP_MODE_PWR_DOWN -the most power savings
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Set sleep mode.
  sleep_enable(); // Enable sleep mode.
  sleep_mode(); // Enter sleep mode.

  // After waking from watchdog interrupt the code continues to execute from this point.

  sleep_disable(); // Disable sleep mode after waking.
  // Re-enable the peripherals.
  //power_all_enable();
}

// 0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms, 5=500ms
// 6=1sec, 7=2sec, 8=4sec, 9=8sec
// From: http://interface.khm.de/index.php/lab/experiments/sleep_watchdog_battery/
void setup_watchdog(int timerPrescaler) {
  if (timerPrescaler > 9 ) timerPrescaler = 9; //Correct incoming amount if need be
  byte bb = timerPrescaler & 7; 
  if (timerPrescaler > 7) bb |= (1<<5); //Set the special 5th bit if necessary
  // enable WatchDog
  //This order of commands is important and cannot be combined
  MCUSR &= ~(1<<WDRF); //Clear the watchdog reset
  WDTCSR |= (1<<WDCE) | (1<<WDE); //Set WD_change enable, set WD enable
  WDTCSR = bb; //Set new watchdog timeout value
  WDTCSR |= _BV(WDIE); //Set the interrupt enable, this will keep unit from resetting after each int
}

// disables the watchdog timer
/*
void disableWatchdog() {
  MCUSR &= ~(1<<WDRF);
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  WDTCSR = 0x00;
}
*/

//Disabling ADC and analog comparator. Saves ~230uA. Needs to be re-enable for the internal voltage check
#define adc_disable() (ADCSRA &= ~(1<<ADEN)) // disable ADC
#define adc_enable()  (ADCSRA |=  (1<<ADEN)) // re-enable ADC

//------------------------------------------------------------------------------

void setup() {
  //Debug serial print only works if DEBUG is defined
  #ifdef DEBUG
    Serial.begin(DEBUGBAUD);
    Serial.println(F("Arduino started"));
  #endif

  //Make sure ESP8266 is set to 9600
  espSerial.begin(ESP8266_BAUD);

  //espSerial.setTimeout(5000);

  //Setup pins
  pinMode(DHT22_POWER_PIN, OUTPUT);
  pinMode(ESP_ENABLE_PIN, OUTPUT);
  digitalWrite(DHT22_POWER_PIN, LOW);
  digitalWrite(ESP_ENABLE_PIN, LOW);

  // 0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms, 5=500ms
  // 6=1sec, 7=2sec, 8=4sec, 9=8sec
  // From: http://interface.khm.de/index.php/lab/experiments/sleep_watchdog_battery/
  setup_watchdog(watchdog_wakeup);      // Wake up after 8 sec

  watchdog_counter = 37; // set to have an initial transmition when starting.

  PRR = bit(PRTIM1);          // only keep timer 0 going
  adc_disable();           // power down/disable the ADC
  analogReference(INTERNAL);   // Set the aref to the internal 1.1V reference

  delay(100);
}

//------------------------------------------------------------------------------

void loop() {
  goToSleep(); // goes to sleep for about 8 seconds and continues to execute code when it wakes up

  if (watchdog_counter >= watchdog_limit) {
    watchdog_counter = 1;

    // Verify enough Supply Voltage
    int vcc = readVcc();
    // stop operation if vcc is lower vccShutdown value (3V)
    if (vcc <= vccShutdown) {
      DEBUG_PRINT(F("ERROR: Supply Voltage is too low:"));
      DEBUG_PRINTLN((vcc / 1000));
      DEBUG_FLUSH();
      // FIXME: power off
      // disable Watchdog Interrupt and power down forever
      wdt_disable();
      return;
    }

    // Hold DHT22 information
    DHT22_ERROR_t errorCode;
    // Power on Sensor
    enableSensor();  
    // Give sensor some time to warmup
    delay(2000);
    // Read DHT22 that will be sent
    errorCode = myDHT22.readData();
    // If DHT22 is not returning valid data
    if (errorCode != DHT_ERROR_NONE) {
      #ifdef DEBUG
        DEBUG_PRINTLN(F("Could not get DHT22 Sensor values. Error:"));
        DEBUG_PRINTLN(errorCode);
      #endif
      disableSensor();
    } else {
      // Prepare string to send
      String msg = "wlanSend(\"";
      msg += "v=";
      msg += vcc;
      msg += "&t=";
      msg += int(myDHT22.getTemperatureCInt()*10);
      msg += "&h=";
      msg += int(myDHT22.getHumidityInt()*10);
      if ((vcc > vccShutdown) && (vcc <= vccNotice)) {
        msg += "&NOTICE=lowVCC";
      }
      msg += "\")";
      //Powerdown Sensor
      disableSensor();

      if (!enableESP()) {
        DEBUG_PRINTLN(F("Error initializing ESP8266!"));
        disableESP();
        DEBUG_PRINTLN(F("Going to sleep..."));
        DEBUG_FLUSH();
        return;
      }

      // Send Data..
      DEBUG_PRINTLN(F("Sending Data.."));
      espSerial.println(msg);
      espSerial.flush();
      DEBUG_PRINTLN(msg);

      // Wait for receiving "DONE"
      if (!waitForString("DONE", 2000)) {
        DEBUG_PRINTLN(F("Error sending Data!"));
      } else {   
        DEBUG_PRINTLN(F("All messages sent!"));
      }

      // Disconnect from AP
      DEBUG_PRINTLN("wlanDisconnect()");
      espSerial.println("wlanDisconnect()");
      espSerial.flush();

      //Disable ESP8266 Module
      disableESP();  
    }
    DEBUG_PRINTLN(F("Going to sleep...\n"));
    DEBUG_FLUSH();
  }
}

// Bring Enable pin up, wait for module to wake-up/connect and connect to AP.
boolean enableESP() {
  //Enable ESP8266 Module
  digitalWrite(ESP_ENABLE_PIN, HIGH);
  DEBUG_PRINTLN(F("ESP8266 Enabled. Waiting to spin up..."));

  //Wait for module to boot up, and connect to AccessPoint
  if (waitForString("READY", 2000)) {
    // Connect to AP
    return connectWiFi();
  }

  //Couldn't connect to the module
  DEBUG_PRINTLN(F("Error initializing the ESP module"));

  //Disable ESP8266 Module
  disableESP();

  //Try again until retry counts expire
  if (retry_attempt < RETRY_COUNT) {
    retry_attempt++;
    delay(500);
    return enableESP();    
  } else {
    retry_attempt = 0;
    return false;
  }
}

// Connect to AP
boolean connectWiFi() {
  // Create connection string and connected to AP
  #ifdef SSID
    String msg = "wlanConnect(\"";
    msg += SSID;
    msg += "\",\"";
    msg += PASS;
    msg += "\")";
  #else
    String msg = "wlanConnect()";
  #endif

  DEBUG_PRINTLN(F("Trying to connect to AP.."));
  espSerial.println(msg);
  espSerial.flush();

  // Wait for receiving "DONE"
  if (!waitForString("> DONE", 10000)) {
    DEBUG_PRINTLN(F("Error connecting to AccessPoint!"));
    return false;
  } else {
    DEBUG_PRINTLN(F("Connected to AccessPoint"));
    return true;
  }
}

// Disable ESP8266 Module
boolean disableESP() {
  digitalWrite(ESP_ENABLE_PIN, LOW);
  DEBUG_PRINTLN(F("ESP8266 Disabled"));
  return true;
}

// Power on Sensor
boolean enableSensor() {
  digitalWrite(DHT22_POWER_PIN, HIGH);
  DEBUG_PRINTLN(F("DHT22 Power ON"));
  return true;
}

// Power off Sensor
boolean disableSensor() {
  digitalWrite(DHT22_POWER_PIN, LOW);
  DEBUG_PRINTLN(F("DHT22 Power OFF"));
  return true;
}

// Wait for specific input string until timeout runs out
boolean waitForString(String input, unsigned int timeout) {
  unsigned long end_time = millis() + timeout;
  while (end_time >= millis()) {
    if (espSerial.available()) {
      while (espSerial.available()) {
        char recieved = espSerial.read();
        inData += recieved;
        if (recieved == '\r') {
          inData.trim();
          DEBUG_PRINT(F("UART:"));
          DEBUG_PRINTLN(inData);
          if (inData == input) {
            inData = ""; // Clear recieved buffer
            return true;
          }
          inData = ""; // Clear recieved buffer
        }
      }
    }
  }
  //Timed out
  return false;
}

//-----------------------------------------------------------------------------
//Code from https://code.google.com/p/tinkerit/wiki/SecretVoltmeter
//-----------------------------------------------------------------------------
int readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  //Re-enable ADC 
  adc_enable();
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
      ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
      ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
      ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
      ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both
  long result = (high<<8) | low;
  //result = 1126400L / result; // Back-calculate Vcc in mV
  result = 1074835L / result;
  //Disable ADC
  adc_disable();
  return result;
}

