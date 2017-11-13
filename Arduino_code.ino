// Car alarm project G3orG3 @2016
// INCLUDING / DEFINE LIBRARYs

#include <Wire.h>
#include <VirtualWire.h>
#include <LiquidCrystal_I2C.h>
#include <LcdBarGraphX.h>
#include <EEPROM.h>
#include <SPI.h>
#include <dht.h>
#include <avr/power.h>
//#include <avr/sleep.h>
#include "LowPower.h"

//ARDUINO PINS:

//internal usage pins
#define buzzerPin 7 // buzzer  ~~~ //
#define buttonPin 24 // button OK ~~~ //
#define voltagePin A15 // car volage ~~~ //
#define lcdon 26 // LCD relay ~~~ //
#define potPin A14 // potentiometer ~~~ //
#define tempPin A13 //temperature PIN ~~~ //

//security
#define doorPin 18  // car's main door sensor  ~~~ //  
#define contactPin 2 // on interrupt
#define alarmTriggerPin 19 //ALARM triggered pin
#define doorlockPin 3 // if the car is locked ~~~
#define alarmPin 40 //ALARM toggler ~~~ //
#define hornPin 34 // horn, lights, car wipers ... ~~~ //
#define lockPin 35 //  pin to the transistor who lock the door ~~~ //
#define unlockPin 36 // pin to the transistor who unlock the door ~~~ //

//Us style lights + interior lights
#define usaRelayPin 47 // the relay used to switch between the normal signal lights and USA lights style mode
#define mainbeamInPin 48 // the status of main beams 
#define signalPin 46 // the status of the signals
#define usaOutPin 10 // the PWM out for USA lights Style
#define ledPin 27 //led strip toggler ~~~ //
#define ledRPin 4 //led RED ~~~ //
#define ledBPin 5 //led BLUE ~~~ //
#define ledGPin 6 //led GREEN ~~~ //

//automatic headlights
#define AutoLPin 25 // RF unlocker toggler ~~~ //
#define uplightSPin A1 // the light sensor for up lighting
#define sidelightSPIN A2 // the light sensor for sides lighting
#define mainbeamPin 9 //the PWM signal for the main beam/parking light
#define lowbeamPin 8 // the PWM signal for the LOW beam

// Settings
int alarmtime = 15; // time for alarm to turn off IN SECOUNDS
int usalight = 50; // % of the lights
int ledlight = 55; //% of the interior light

int redVal = 0;   // Variables to store the values to send to the pins
int grnVal = 0;
int bluVal = 0;
int sleepStatus = 0;             // variable to store a request for sleep
int ledToggler = 0; //led toggler status
int alarmToggler = 0; //alarm toggler status
int menupage = 0; // menu page
int menusel = 0; // menu selected item
int menumode = 0;
int menudone = 0; //the value of the setting that must be applied
int before = 0; //used to enter settings old pot
int beforepot = 0; //used to navigate in homepage old pot
int cur = 0;
int count = 0;
int idlecount = 0;
int potVal = 0;
int settingspos = 0; //cursor pos for settings
int lcdf = 0; //used for lcd clearing function
int alarmOn = 0; //alarm triggered

int sleep_querty = 0;
int sleep_delay = 0; // the delay before the arduino enters sleep mode

int lcdstatus = 1; // 1-on 0 off
int hornstatus = 0;  // 1-on 0 off
int waitba = 0;
int wakeup = 0;
int beforebat = 0;
int doorwarning = 0; // used to warn if the door is opened when it tries to lock the car
int multipage = 0; // used to switch pages in the status homepage
int carstatus = 0; // the car status // ON OR OFF

int ledi_status;
int ledi_dim;
int ledi_timeout;

int usaComeback = 0;
int ledu_timeout;

int ledu_status;
int ledu_dim;
int relayOn;


int ledl_status;
int ledl_dim;
int ledl_timeout;
int autoLightStatus;

int ledp_status;
int ledp_dim;
int ledp_timeout;

int vdelay = 0; //virtual delay

int locksts;

int blink_done = 1;
int blink_time;
int blink_count;

int horn_done = 1;
int horn_time;
int horn_count;

int tempC_full = 0;
int tempC = 0;
int tempC_print = 0;

unsigned long last_millis_all;
unsigned long last_millis_s;
unsigned long last_millis_a;
unsigned long last_millis_b;
unsigned long last_millis_fadefoot;
unsigned long last_millis_fadeusa;
unsigned long last_millis_fadebeam;
unsigned long last_millis_fadepark;


int alarm_allow;
int alarm_count_current;
int alarm_allow_cooldown;
unsigned long alarm_count_millis;

int alarmCD = 10000;

int doorlock_cooldown;

boolean doorlock;
boolean button;
boolean signall;
boolean door;
boolean contact;
boolean radarTrigger;
boolean mainbeam_on;
boolean engine_on;
int mainbeam_on_count;
int contact_count;

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address -> lcd_Addr, EN, RW, RS, D4, D5, D6, D7

//custom car animation
byte off[8] = {
  0b00000,
  0b10001,
  0b01010,
  0b00100,
  0b01010,
  0b10001,
  0b00000,
  0b00000
};
byte on[8] = {
  0b00000,
  0b00001,
  0b00010,
  0b10100,
  0b11000,
  0b10000,
  0b00000,
  0b00000
};

byte none[8] = {
  0b00000,
  0b00000,
  0b01110,
  0b01110,
  0b01110,
  0b01110,
  0b00000,
  0b00000
};


#define NOTE_G7  5136
#define NOTE_E7  4637
#define NOTE_D7  4349
#define NOTE_C7  4093
#define NOTE_E6  3319
#define NOTE_B5  2988

LcdBarGraphX lbg(&lcd, 7, 9, 1);
LcdBarGraphX lbg1(&lcd, 7, 0, 1);
LcdBarGraphX lbg2(&lcd, 16, 0, 1);
LcdBarGraphX lbg3(&lcd, 16, 0, 0);

dht DHT;


#define DEBUG /// TOGGLE DEBUG MODE


#ifndef DEBUGUTILS_H
#define DEBUGUTILS_H

#ifdef DEBUG
#define DEBUG_PRINT(...) Serial.println(__VA_ARGS__)
#define DEBUG_START() Serial.begin(19200)
#else
#define DEBUG_PRINT(...)
#define DEBUG_START()
#endif

#endif

void setup() {

  DEBUG_START();
  DEBUG_PRINT("Starting setup");

  //SET UP ARDUINO PINS (13 declarated)
  pinMode(ledRPin, OUTPUT);
  pinMode(ledGPin, OUTPUT);
  pinMode(ledBPin, OUTPUT);
  pinMode(potPin, INPUT);
  pinMode(voltagePin, INPUT);
  pinMode(contactPin, INPUT);
  pinMode(doorPin, INPUT_PULLUP);
  pinMode(alarmTriggerPin, INPUT);  //wake up / interrupt
  pinMode(lockPin, OUTPUT);
  pinMode(unlockPin, OUTPUT);
  pinMode(hornPin, OUTPUT);
  // pinMode(truckopenPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(alarmPin, INPUT_PULLUP);
  pinMode(AutoLPin, INPUT_PULLUP);
  pinMode(ledPin, INPUT_PULLUP);
  pinMode(lcdon, OUTPUT);
  pinMode(usaRelayPin, OUTPUT);
  pinMode(mainbeamInPin, INPUT);
  pinMode(signalPin, INPUT);
  pinMode(usaOutPin, OUTPUT);
  pinMode(doorlockPin, INPUT);  //wake up / interrupt
  pinMode(mainbeamPin, OUTPUT);
  pinMode(lowbeamPin, OUTPUT);
  pinMode(uplightSPin, INPUT);
  pinMode(sidelightSPIN, INPUT);

  DEBUG_PRINT("Pins Done!");

  // while (check_battery() <= 9) {
  //  delay(1500);
  //  tone(buzzerPin, NOTE_B5, 60);
  //  delay(900);
  // }
  DEBUG_PRINT("Battery check");
  delay(10);

  //buzzer test
  tone(buzzerPin, NOTE_G7, 100);
  DEBUG_PRINT("Buzzer test");

  radarTrigger = false;

  //wake up + initialize LCD
  digitalWrite(lcdon, HIGH);
  delay(100); //wait relay
  DEBUG_PRINT("LCD Power!");
  lcd.begin(16, 2);              // initialize the lcd
  DEBUG_PRINT("LCD Setup!");
  lcd.createChar(7, on);  //create custom car
  lcd.createChar(8, off); //create custom car
  lcd.createChar(9, none); //create custom car
  lcdstatus = 1;
  DEBUG_PRINT("LCD started!");

  // set rgb led strip to 0
  analogWrite(ledRPin, 0);
  analogWrite(ledGPin, 0);
  analogWrite(ledBPin, 0);
  ledi_dim = 100;
  ledi_timeout = 5000;
  ledi_status = 0;
  DEBUG_PRINT("Footlights setup");

  // usa lights PWM 0
  analogWrite(usaOutPin, 0);
  ledu_dim = 0;
  ledu_status = 0;
  ledu_timeout = 5000;
  relayOn = 0;
  TCCR2B = (TCCR2B & 0b11111000) | 0x02; //default:0x04
  TCCR4B = (TCCR4B & 0b11111000) | 0x02; //default:0x04 normal:0x06
  ledl_dim = 0;
  ledl_status = 0;
  ledl_timeout = 5000;
  ledp_dim = 0;
  ledp_status = 0;
  ledp_timeout = 5000;
  analogWrite(lowbeamPin, 0);
  analogWrite(mainbeamPin, 0);
  DEBUG_PRINT("US LIGHTS setup");

  // RESTORE SETTINGS
  redVal =   EEPROM.read(0);
  grnVal =   EEPROM.read(1);
  bluVal =   EEPROM.read(2); //colors
  usalight =   EEPROM.read(8); //USA lights opacity
  ledlight =   EEPROM.read(9); //interior lights opacity
  DEBUG_PRINT("Settings restored!");

  alarm_allow = 1; // allow alarms for the first time

  DEBUG_PRINT("Interrupt:");
  attachInterrupt(digitalPinToInterrupt(doorlockPin), doorlockPin_change, CHANGE);

  DEBUG_PRINT("Done setup!");

}


///////////////////////////////////////////////////////////////////////////////////////
void click_sound() {
  tone(buzzerPin, NOTE_B5, 100);
}

void error_sound() {
  tone(buzzerPin, NOTE_E6, 1000);
}

void short_sound() {
  tone(buzzerPin, NOTE_G7, 20);
}


///////////////////////////////////////////////////////////////////////////////////////
void unlock_door()
{
  DEBUG_PRINT("Trying to UNlock the door");
  // if the door in not opened
  if (door == true) { //show only after the door is opened ! // doorPin = 0 -> usa deschisa
    DEBUG_PRINT("Sended UnLock command !");
    idlecount = 0;
    digitalWrite(unlockPin, HIGH);
    short_sound();
    delay(200); //default 200
    short_sound();
    digitalWrite(unlockPin, LOW);
    delay(150); //default 200
    alarmCD = 10000;                      //alarm cooldown after the car is locked
    unlock_lights(); // turn on the signal lights + TIMEOUT
  }
  else { //if the door is closed
    DEBUG_PRINT("Cannot UNlock the door");
    lcd_msg_add("Usa principala", "NU este inchisa!");

    short_sound();
  }
}


///////////////////////////////////////////////////////////////////////////////////////
void lock_door()
{
  DEBUG_PRINT("Trying to lock the door");
  // if the door in not opened
  if (door == true) { //show only after the door is closed! // doorPin = 0 -> usa deschisa   1 -> inchisa
    DEBUG_PRINT("Sended Lock command !");
    idlecount = 0;
    digitalWrite(lockPin, HIGH);
    short_sound();
    delay(200); //default 200
    short_sound();
    digitalWrite(lockPin, LOW);
    delay(150); //default 200

    alarmCD = 10000;                      //alarm cooldown after the car is locked
    unlock_lights(); // turn on the signal lights
  }
  else { //if the door is closed
    DEBUG_PRINT("Cannot lock the door");
    lcd_msg_add("Usa principala", "NU este inchisa!");
    short_sound();
  }
}


///////////////////////////////////////////////////////////////////////////////////////
void alarm_do() {
  //////////////////////// ALARM turning ON
  if (alarmOn == 0 ) {         //if the alarm is not already ON
    if (alarmToggler == 1) {               //if the alarm is toggler enabled
      if (doorlock == true) {            //if the car is locked
        if (alarmCD == 0) {              //if the cooldown is DOWN
          if (radarTrigger == true) {    //if Radar detected presence
            if (alarm_allow != 0) {
              DEBUG_PRINT("alarm_do ON");
              alarm_count_current++;
              alarmOn = 1;               //set alarm as triggered
              horn_on(20000); //turn the horn on for the alarm time
            } else {
              DEBUG_PRINT("Alarm not allowed !");
              DEBUG_PRINT(alarm_allow_cooldown);
            }
          }
        } else {
          alarmCD--;                     //decrease the cooldown
          if ((alarmCD % 100) == 0) {     //DEBUG
            DEBUG_PRINT("Alarm CD");     //DEBUG
            DEBUG_PRINT(alarmCD);       //DEBUG
          }                             //DEBUG
        }
      }
    }
  } else if (alarmOn == 1) {    //if the alarm is already triggered
    if (horn_count >= 3000 || horn_count == 0 ) {  //if the key is near or timeout is OUT
      DEBUG_PRINT("alarm_do OFF");
      alarmOn = 0;                                   //set alarm flag off
      alarmCD = 2500;                                //set alarm CoolDown to 50*50ms (preventing alarm spam)
      horn_on(1);
      error_sound();
    } else {
      if ((horn_count % 50) == 0) {
        DEBUG_PRINT("Horn count:");    //DEBUG
        DEBUG_PRINT(horn_count);       //DEBUG
      }
      //  if (horn_count == 0) {
      //if the program is stucked!
      //    alarmOn = 0;                  //set alarm flag off
      //    alarmCD = 100;                //set alarm CoolDown to 50*50ms (preventing alarm spam)
      //    horn_done = 1;                                 //force horn to turn off
      //    DEBUG_PRINT("horn at 0 count!");
      //  }
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////
void lcd_on() {
  if (lcdstatus != 1) {
    DEBUG_PRINT("LCD ON");
    idlecount = 0;
    power_twi_enable();
    digitalWrite(lcdon, HIGH);
    delay(500);
    lcd.begin(16, 2);  // initialize the lcd for 16 chars 2 lines, turn on backlight
    lcd.createChar(7, on);  //create custom car
    lcd.createChar(8, off); //create custom car
    lcd.createChar(9, none); //create custom car
    delay(50);
    lcdstatus = 1;
  }
}

///////////////////////////////////////////////////////////////////////////////////////
void lcd_off() {
  if (lcdstatus != 0) {
    DEBUG_PRINT("LCD OFF");
    idlecount = 0;
    power_twi_disable();
    delay(100);
    digitalWrite(lcdon, LOW);
    power_twi_disable();
    lcdstatus = 0;
  }
}

void wakeUp()
{
}


///////////////////////////////////////////////////////////////////////////////////////
void go_sleep() {
  DEBUG_PRINT("go_sleep");
  click_sound();
  idlecount = 0;
  menumode = 0;
  sleep_on();
  delay(500);
}

void sleep_on() { //use sleepStatus == 1; to use it
  if (sleep_delay == 0) {

    if (sleepStatus == 1) {
      DEBUG_PRINT("sleep_on PASS");
      error_sound();
      delay(500);
      error_sound();
      delay(250);
      error_sound();
      delay(100);

      //power down USA LIGHTS
      analogWrite(ledBPin, LOW);
      digitalWrite(usaRelayPin, LOW); // relay rail to PWM
      ledu_dim = 0;
      ledu_status = 0;
      relayOn = 0;

      //power down main/low beams
      analogWrite(lowbeamPin, LOW);
      analogWrite(mainbeamPin, LOW);
      ledp_dim = 0;
      ledp_status = 0;
      ledl_dim = 0;
      ledl_status = 0;


      //shut down functions
      analogWrite(ledRPin, LOW);   // Write values to LED pins
      analogWrite(ledGPin, LOW);
      analogWrite(ledBPin, LOW);
      ledi_dim = 0;
      ledi_status = 0;

      //power down LCD
      lcd_off();

      delay(60); // wait a bit

      // Allow wake up pin to trigger interrupt on high
      attachInterrupt(digitalPinToInterrupt(doorPin), wakeUp, CHANGE);
      attachInterrupt(digitalPinToInterrupt(doorlockPin), wakeUp, CHANGE);
      attachInterrupt(digitalPinToInterrupt(contactPin), wakeUp, CHANGE);
      if (alarmToggler == 1) { //if function is active
        attachInterrupt(digitalPinToInterrupt(alarmTriggerPin), wakeUp, CHANGE);
      }

      LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); //OR powerSave

      //  sleep_mode();            // here the device is actually put to sleep!!
      // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP

      // sleep_disable();         //first thing after waking from sleep
      alarmCD = 0;             //reset the cooldown to alarmCD / ready to trigger the alarm
      sleepStatus = 0;         //set the flag for sleep status
      idlecount = 0;           //reset the sleep counter

      //remove all interrupts
      detachInterrupt(digitalPinToInterrupt(contactPin));
      detachInterrupt(digitalPinToInterrupt(doorPin));
      if (alarmToggler == 1) { //if function is active
        detachInterrupt(digitalPinToInterrupt(alarmTriggerPin));
      }
    }
  }
  else {
    sleep_delay = sleep_delay - 1;
  }
  // }
}


int doorlock_wave = 0;
int doorlock_onetime = 0;
int doorlock_timer;
int doorlock_step;

int doorlock_temp = 0;

void doorlockPin_change()
{
  doorlock_temp = true;
}



///////////////////////////////////////////////////////////////////////////////////////
void signalswatch() {
  if (contact == true) { //if the car is ON
    if (signall == true) { //SWITCH TO SIGNALS !!!!!!!!!! VERY IMPORTANT TO BE STABLE~~~~~~~~~~~~~~~~
      if (relayOn != 0) { //if the relay is not ALREADY OFF
        DEBUG_PRINT("SignalSwitch Relay ON SIGNAL");
        digitalWrite(usaRelayPin, LOW);
        relayOn = 0;
      }
      if (ledu_status >= 2) {
        DEBUG_PRINT("Turning OFF US LIGHTS0");
        ledu_status = 1;                      //turn OFF the usa lights
      }
      usaComeback = 8; // x*49ms delay = time // LAST VALUE 12
    } else {                                       //if the signal is not ON
      if (usaComeback <= 0) {                    //if there is no cooldown
        if (engine_on == true) {
          if (relayOn != 1) {
            DEBUG_PRINT("SignalSwitch Relay ON US LIGHTS");
            digitalWrite(usaRelayPin, HIGH);    //relay rail to PWM
            relayOn = 1;
          }
          if (mainbeam_on == true && ledToggler == 1) {  //if main beams are ON
            if (ledu_status <= 1) {
              DEBUG_PRINT("Turning on US LIGHTS1");
              ledu_status = 2;                    //turn ON the usa lights
            }
          } else {                                         // if main beams are OFF
            if (ledu_status >= 2) {
              DEBUG_PRINT("Turning OFF US LIGHTS2");
              ledu_status = 1;                      //turn OFF the usa lights
            }
          }
        }
      } else {
        usaComeback = usaComeback - 1;
      }
    }
  } else {
    if (ledu_status >= 2) {
      DEBUG_PRINT("Turning OFF US LIGHTS3");
      ledu_status = 1;                      //turn OFF the usa lights
    }
  }
}



///////////////////////////////////////////////////////////////////////////////////////
float check_battery()
{
  //READ Battery voltage
  float vbat = analogRead(voltagePin);
  float temp222 = (vbat * (5.02 / 1023.0));
  float temp2222 = (temp222 / 0.2323);
  return temp2222;
}


///////////////////////////////////////////////////////////////////////////////////////

void unlock_lights() {
  alarmCD = 10000;                      //alarm cooldown after the car is locked
  if (relayOn != 1) {                 //if the relay is OFF
    DEBUG_PRINT("Move relay to us lights");
    digitalWrite(usaRelayPin, HIGH);  //relay rail to PWM
    relayOn = 1;
    delay(10);
  }

  //initiate blinking effects with no delay
  blink_done = 0;
  blink_time = 0;
  blink_count = 0;
  //alarmCD time = x*50ms // 200 steps = 10s

}


///////////////////////////////////////////////////////////////////////////////////////
void lights_blink_fade() {
  if (blink_done != 1) {             //if you didn't blinked already
    if (blink_count <= 6) {          //if you not finished your blinking number
      if (blink_count == 0) {        //if you not finished your blinking number
        if (doorlock == true) {
          horn_on(80);                 //set the horn on for 50ms during on lights (matching the blinking time)
        }
        if (doorlock == false) {
          horn_on(110);                 //set the horn on for 50ms during on lights (matching the blinking time)
        }
      }
      if ((blink_count % 2) == 0) {  //if the blinking number is ODD  //EXAMPLE : 0-ON , 1-OFF, 2-ON, 3-OFF, 4-ON
        if (doorlock == true) analogWrite(lowbeamPin, 180);
        if (doorlock == false) analogWrite(usaOutPin, 250);
        delay(3);
      } else {
        if (doorlock == true) analogWrite(lowbeamPin, 0);
      }
      blink_count++;
    } else {
      DEBUG_PRINT("lights_blink_fade END PASS");
      analogWrite(lowbeamPin, 0);
      blink_done = 1;                //set the flag for blinking is done
      blink_time = 0;
      blink_count = 0;
    }
  }
}


///////////////////////////////////////////////////////////////////////////////////////

int lcd_msg_tleft = 50; //the maximum time
boolean lcd_msg_printing = false; //

String lcd_msg_querty1[10]; //10 messages maximum
String lcd_msg_querty2[10]; //10 messages maximum

void lcd_msg_add(String line1, String line2) {
  for (int i = 0; i <= 9; i++) { //search in all the empty querty
    if (lcd_msg_querty1[i] == NULL) { //find the empty querty space
      if (line1.length() < 17) {
        if (line2.length() < 17) {
          lcd_msg_querty1[i] = line1; //add string to querty
          lcd_msg_querty2[i] = line2;
          DEBUG_PRINT("Printing added " + String(i));
          break; //add it just one time!
        }
      }
    }
  }
}

void lcd_msg() {

  if (lcdstatus == 1) {

    if (lcd_msg_printing == false) {
      for (int i = 0; i <= 9; i++) {                //search in all the querty
        if (lcd_msg_querty1[i] != NULL) {        //search for not null strings
          DEBUG_PRINT("Detected string in print querty" + String(i));
          lcd.clear();
          String line1 = lcd_msg_querty1[i];    //save the string from the querty
          lcd_msg_querty1[i] = "";              //delete the string from the querty
          String line2 = lcd_msg_querty2[i];
          lcd_msg_querty2[i] = "";
          DEBUG_PRINT("Printing line: " + String(line1) + String(line2));
          lcd.setCursor(((16 - line1.length()) / 2), 0); //print text centered
          lcd.print(line1);
          lcd.setCursor(((16 - line2.length()) / 2), 1); //print text centered
          lcd.print(line2);
          lcdf = 1; //don't print menu over
          lcd_msg_printing = true;               //mark it as printed
          break;                      //print it just one time!
        }
      }
    }

    if (lcd_msg_printing == true) {
      lcd_msg_tleft--;
      if (lcd_msg_tleft % 20 == 0) DEBUG_PRINT("Printing time left: " + String(lcd_msg_tleft));
    }

    if (lcd_msg_tleft <= 0) {
      lcd_msg_printing = false; //ready for another print
      lcd_msg_tleft = 50;
      lcdf = 0; //can print menu over
      DEBUG_PRINT("Printing done");
    }

  } else { //if the lcd if off
    lcd_on();
  }

}

///////////////////////////////////////////////////////////////////////////////////////
void horn_on(int x) {
  // x = time of the horn in ms
  // usage: horn_on(100);  where 100 = 100ms time for horn
  DEBUG_PRINT("HORN ON FOR: ");
  DEBUG_PRINT(x);
  horn_time = x;
  horn_count = 0;
  horn_done = 0;
  hornstatus = 0;
}

void horn_do() {
  // TO FORCE STOP THE HORN USE: horn_done = 1;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  if (horn_done != 1) {             //if you didn't blinked already
    if (horn_count <= horn_time) {
      if (hornstatus != 1) {
        digitalWrite(hornPin, HIGH);//horn on
        tone(buzzerPin, NOTE_B5);
        DEBUG_PRINT("Sended Horn ON command!");
        hornstatus = 1;             //set horn status to ON
      }
    }
    else {                         //if you finished blinking
      digitalWrite(hornPin, LOW);  //horn off
      noTone(buzzerPin);
      DEBUG_PRINT("Sended Horn OFF command!");
      hornstatus = 0;              //set horn status to OFF
      horn_done = 1;
      horn_time = 0;            //set the flag for horn is done
      horn_count = 0;
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////
void alarm_protect()
{
  if (alarm_count_current == 1) {
    alarm_count_millis = millis();
  }
  else if (alarm_count_current >= 1) {
    unsigned long temp_math = millis() - alarm_count_millis;
    if (temp_math >= 100000) {    //if the interval is lower then 100s
      alarm_count_millis = 0;     //reset the interval
      alarm_count_current = 0;    //reset the counter
    }
    if (alarm_count_current >= 10) {
      //
      //   determine the interval between the first alarm and the maximum alarm counter
      //
      //   current alarm time is 4s
      //   maximum interval is 100s
      //   maximum alarm counter is 25
      //   protection allow only first 10 alarms
      //
      //   result: if the alarm is triggered more then 10 times in maximum of 100s then lock the alarm for 10 seconds
      if (temp_math <= 100000) {    //if the interval is lower then 100s
        alarm_allow = 0;
      }
      else {                        //if the interval is bigger (alarm not spammed)
        alarm_count_millis = 0;     //reset the interval
        alarm_count_current = 0;    //reset the counter
      }
    }
  }

  if (alarm_allow == 0) {
    alarm_allow_cooldown++;
    if (alarm_allow_cooldown >= 10000) { // 10 seconds alarm cooldown
      alarm_allow = 1;
    }
  }

}

///////////////////////////////////////////////////////////////////////////////////////

boolean need_beam;
/////////////////CALIBRATOR
//TOTAL MUST BE 100%
int upS_perc = 50;
int sideS_perc = 50;

int dark_light = 900;
int bright_light = 920;

int readlight_maxcount = 10;
int nobeam_maxcount_delay = 16;
int beam_maxcount_delay = 2;
/////////////////END CALIBRATOR

int nobeam_count_delay;
int beam_count_delay;

int readlight_count;

int totalupS;
int totalsideS;

int last_light_read;

boolean engine_just_started;
int engine_just_started_time = 8000;
boolean last_light_blink;


//////////////////////////////////////////////////////////////////////////////////
void readlightsensor() {
  int upS = analogRead(uplightSPin);                       //read the current sensor data
  int sideS =  analogRead(sidelightSPIN);                  //read the current sensor data

  if (upS >= 0 && sideS >= 0) {
    totalupS = totalupS + upS;                 //SUM the readings
    totalsideS = totalsideS + sideS;
    readlight_count++;                         //count the readings
  }


  if (readlight_count >= readlight_maxcount) {             //if the COUNT reading reach it's maximum

    totalupS = totalupS / readlight_maxcount;              //AVERAGE THE SUM
    totalsideS = totalsideS / readlight_maxcount;          //AVERAGE THE SUM

    //calculate the priority
    //  int allS = ((totalupS * upS_perc) / 100) + ((totalsideS * sideS_perc) / 100);
    int allS = ((totalupS + totalsideS) / 2);

    // if (allS <= 0) allS = last_light_read;                 //negative number filtrer

    last_light_read = allS;                                //Store the reading for printing

    totalupS = 0;                                          //RESET the SUM
    totalupS = 0;                                          //RESET the SUM
    readlight_count = 0;                                   //RESET THE COUNT

    if (allS >= bright_light) {                            //if there is already light detected
      nobeam_count_delay++;                               //make sure we really don't need light
      if (nobeam_count_delay >= nobeam_maxcount_delay) {   //count how many times you don't need light
        need_beam = false;                                 //finnaly turn the light off
        nobeam_count_delay = 0;
        beam_count_delay = 0;
      }
    }

    if (allS <= dark_light) {
      beam_count_delay++;          //make sure we really don't need light
      if (beam_count_delay >= beam_maxcount_delay) {   //count how many times you don't need light
        need_beam = true;                                 //finnaly turn the light off
        nobeam_count_delay = 0;                         //finnaly turn the light off
        beam_count_delay = 0;
      }
    }

  }
}

//////////////////////////////////////////////////////////////////////////////////////

void autofootlights() {
  boolean final_fl = false;
  if (door == true && ledToggler == 1) {
    final_fl = true;
  }
  if (contact == true && engine_on == true) {      //if the car is on
    final_fl = true;
  }

  if (ledi_timeout <= 0 && contact == false) {                // IF TIMEOUT IS OUT !
    final_fl = false;
  }

  if (ledi_timeout > 0 && contact == false && engine_on == false) {
    ledi_timeout--;  //count down the led timeout
  }

  if (ledToggler == 0) {             // if the toggler is OFF
    final_fl == false;
  }

  if (final_fl == true) {
    if (ledi_status <= 1) {          //if the footlights are OFF
      ledi_status = 2;               //turn on the lights
      ledi_timeout = 6000;           //close lights after 6000ms after the door is closed
    }
  } else {

    if (ledi_status >= 2) {          //if the footlights are ON
      ledi_status = 1;               //turn OFF the lights
      ledi_timeout = 0;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////

void autolowbeam() {
  if (engine_just_started == false) {
    if (contact == true && need_beam == true && autoLightStatus == 1 && engine_on == true) {     //if the contact is ON /AND/ the sensor indicates DARK
      if (ledl_status <= 1) {                       //if the LOW BEAM is NOT already ON
        ledl_status = 2;                            //TURN ON the LOW BEAM !
        lcd_msg_add("Se Aprind", "Farurile !");
        DEBUG_PRINT("Turning on low beams");
      }
    }
    if (contact == false || need_beam == false || autoLightStatus == 0 || engine_on == false) {
      if (ledl_status >= 2) {                       //if the LOW BEAM is NOT already ON
        ledl_status = 1;                            //TURN OFF the LOW BEAM !
        lcd_msg_add("Se Sting", "Farurile !");
        DEBUG_PRINT("Turning OFF low beams");
      }
    }
  }
}


///////////////////////////////////////////////////////////////////////////////////////
void autoparkinglight() {
  if (autoLightStatus == 1) {         //if the toggler is ON
    //if the contact is ON AND the engine si ON too AND the toggler is On too!
    if (contact == true && engine_on == true && autoLightStatus == 1) {
      if (ledp_status <= 1) {         //if the parking light is NOT already ON
        ledp_status = 2;              //TURN ON the parking light !
      }
    }
    if (contact == false || engine_on == false || autoLightStatus == 0) {                         //if the contact is OFF or the engine is OFF or the toggler is OFF
      if (ledp_status >= 2) {         //if the parking light is NOT already OFF
        ledp_status = 1;              //TURN OFF the parking light !
      }
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////

void loop()
{



  if (millis() - last_millis_all >= 1) {
    last_millis_all = millis();

    // DEBUG ONLY
    if (idlecount % 100 == 0) DEBUG_PRINT("Idle counter: " + String(idlecount));      //DEBUG

    /////////////////////// ENGINE OFF / CONTACT ON DETECTION!
    if (engine_just_started == false) {
      if (check_battery() >= 13.5) {        //flag engine as started after reaching 14 Volts
        if (engine_on != true) {
          engine_on = true;
          DEBUG_PRINT("Engine ON");      //DEBUG
        }
      } else if (check_battery() < 12.8 && contact == false) {    //if the battery drops below 13 Volts
        if (engine_on != false) {
          engine_on = false;         //flag engine as OFF
          DEBUG_PRINT("Engine OFF");      //DEBUG
        }
      }
    }

    if (engine_just_started == true) {
      if (engine_just_started_time > 0) {
        engine_just_started_time--;
        if (engine_just_started_time % 500 == 0) {
          last_light_blink = false;
        }

        if (engine_just_started_time % 513 == 0) {
          last_light_blink = true;
        }

        int lights_value = 0;
        if (last_light_blink == true) lights_value = 220; /////config
        analogWrite(lowbeamPin, lights_value);       //turn off the lights
        ledl_dim = 255;                             //trick the lights to stay off
        ledl_status = 3;
      }
      if (engine_just_started_time == 0 || engine_on == true) {
        DEBUG_PRINT("Engine on Blinking OFF : engine /time:" + String(engine_on) + String(engine_just_started_time));            //DEBUG
        last_light_blink = false;
        engine_just_started = false;
        engine_just_started_time = 8000;
        analogWrite(lowbeamPin, 0);       //turn off the lights
        ledl_dim = 0;                             //trick the lights to stay off
        ledl_status = 0;
        click_sound();
      }
    }

    if (blink_done != 1) {        //if blinking isn't finished
      blink_time = blink_time + 1;
      if ((blink_time % 200) == 0) {
        lights_blink_fade();
      }
    }

    if (horn_done != 1) {       //if horn isn't finished
      horn_count++;
      horn_do();
    }

    if (doorlock_cooldown >= 1) { //if the doorlock cooldown isn't finished
      doorlock_cooldown--;
      if ((doorlock_cooldown % 100) == 0) {   //DEBUG
        DEBUG_PRINT("DoorL CD: ");            //DEBUG
        DEBUG_PRINT(doorlock_cooldown);      //DEBUG
      }                                      //DEBUG
    }

    alarm_do();           //Decide the alarm
    alarm_protect();      // alarm spam protection

  }


  /////////////////////// Doorlock JOB
  if ((millis() - last_millis_b) >= 150) {
    last_millis_b = millis();
    if (doorlock_step == 0) {           //if this is the first step
      doorlock_temp = false;            //drop the bait
      doorlock_step++;                  //set this step as done
    }
    else {                              //if this is the second step
      if (doorlock_temp == false) {     //if the bait wasn't bitted
        if (doorlock != true) {
          doorlock = true;              //this means the car is locked
          unlock_lights();
          DEBUG_PRINT("~~Car is now Locked!");

        }
      }
      else {
        if (doorlock != false) {
          doorlock = false;
          horn_count = 0;
          unlock_lights();
          DEBUG_PRINT("~~Car is now UnLocked!");
        }
      }
      doorlock_step = 0;
    }
  }


  ////////////////////// FADE EFFECT FOR LOW BEAM
  if (millis() - last_millis_fadebeam >= 22) {
    last_millis_fadebeam = millis();
    if (ledl_status == 2) { //TURNING ON ~~~~~~~~~
      if (ledl_dim != 255) {
        ledl_dim++;
        analogWrite(lowbeamPin, ledl_dim);
      }
      else {
        ledl_status = 3;
        analogWrite(lowbeamPin, 255);
        short_sound();
      }
    }
    if (ledl_status == 1) { //TURING OFF ~~~~~~~~~~
      if (ledl_dim != 0) {
        ledl_dim--;
        analogWrite(lowbeamPin, ledl_dim);
      }
      else {
        ledl_status = 0;
        click_sound();
      }
    }
  }

  ////////////////////// FADE EFFECT FOR MAIN BEAM/parking lights
  if (millis() - last_millis_fadepark >= 22) {
    last_millis_fadepark = millis();
    if (ledp_status == 2) { //TURNING ON ~~~~~~~~~
      if (ledp_dim != 255) {
        ledp_dim++;
        analogWrite(mainbeamPin, ledp_dim);
      }
      else {
        ledp_status = 3;
        short_sound();
        analogWrite(mainbeamPin, 255);
      }
    }
    if (ledp_status == 1) { //TURING OFF ~~~~~~~~~~
      if (ledp_dim != 0) {
        ledp_dim--;
        analogWrite(mainbeamPin, ledp_dim);
      }
      else {
        ledp_status = 0;
        click_sound();
      }
    }
  }


  ////////////////////// FADE EFFECT FOR USA LIGHTS
  if (millis() - last_millis_fadeusa >= 14) {
    last_millis_fadeusa = millis();
    if (ledu_status == 2) { //TURNING ON ~~~~~~~~~
      if (ledu_dim != map(usalight, 0, 100, 0, 255)) {
        ledu_dim++;
        analogWrite(usaOutPin, ledu_dim);
      }
      else {
        ledu_status = 3;
      }
    }
    if (ledu_status == 1) { //TURING OFF ~~~~~~~~~~
      if (ledu_dim != 0) {
        ledu_dim--;
        analogWrite(usaOutPin, ledu_dim);
      }
      else {
        ledu_status = 0;
      }
    }
  }

  ////////////////////// FADE EFFECT FOR FOOTLIGHTS
  if (millis() - last_millis_fadefoot >= 30) {
    last_millis_fadefoot = millis();

    if (ledi_status == 2) {
      if (ledi_dim != 0) {
        ledi_dim--;
        int redled = ((redVal - ((redVal * ledlight) / 100)) * (100 - ledi_dim) / 100);
        analogWrite(ledRPin, redled);
        int grnled = ((grnVal - ((grnVal * ledlight) / 100)) * (100 - ledi_dim) / 100);
        analogWrite(ledGPin, grnled);
        int bluled = ((bluVal - ((bluVal * ledlight) / 100)) * (100 - ledi_dim) / 100);
        analogWrite(ledBPin, bluled);
      }
      else {
        ledi_status = 3;
      }
    }
    if (ledi_status == 1) {     //TURING OFF ~~~~~~~~~~
      if (ledi_dim != 100) {
        ledi_dim++;
        int redled = ((redVal - ((redVal * ledlight) / 100)) * (100 - ledi_dim) / 100);
        analogWrite(ledRPin, redled);
        int grnled = ((grnVal - ((grnVal * ledlight) / 100)) * (100 - ledi_dim) / 100);
        analogWrite(ledGPin, grnled);
        int bluled = ((bluVal - ((bluVal * ledlight) / 100)) * (100 - ledi_dim) / 100);
        analogWrite(ledBPin, bluled);
      }
      else {
        ledi_status = 0;
      }
    }
  }


  if (millis() - last_millis_a >= 50) {
    last_millis_a = millis();


    button = digitalRead(buttonPin);

    ///////////////////// DOOR STATUS READ
    if (digitalRead(doorPin) == HIGH) {
      if (door != true) {
        DEBUG_PRINT("~~~~Door Closed!");
        DEBUG_PRINT(door);
        door = true; //true
      }
    }
    else {
      if (door != false) {
        DEBUG_PRINT("~~~~Door Opened!");
        door = false;
      }
    }

    ///////////////////// DOOR STATUS READ
    if (digitalRead(signalPin) == HIGH) {
      if (signall != true) {
        DEBUG_PRINT("~~~~signall ON!");
        signall = true; //true
      }
    }
    else {
      if (signall != false) {
        DEBUG_PRINT("~~~~signall OFF!");
        signall = false;
      }
    }

    // DEBUG_PRINT("MAINBEAM: " + String(digitalRead(mainbeamInPin)));

    // if (ledp_status == 0 || ledp_status == 3) {
    if (digitalRead(mainbeamInPin) == HIGH ) { // || ledp_status == 3) {
      if (mainbeam_on != true) {
        mainbeam_on_count++;
        if (mainbeam_on_count >= 30) {
          mainbeam_on = true;
          DEBUG_PRINT("MainBeam deected ON!");
          mainbeam_on_count = 0;
        }
      }
    } else if (digitalRead(mainbeamInPin) == LOW) {
      if (mainbeam_on != false) {

        mainbeam_on_count = 0;
        mainbeam_on = false;
        DEBUG_PRINT("MainBeam deected OFF!");
      }
    }
    //  }



    ///////////////////// CONTACT STATUS READ
    if (engine_just_started == false) {
      if (digitalRead(contactPin) == HIGH) {
        if (contact != true) {
          DEBUG_PRINT("~~~~Contact On!");
          idlecount = 0;
          relayOn = 0;
          engine_just_started = true;
          contact = true;
          contact_count = 0;
          beforebat = check_battery();             //save the voltage of the car
          lcd_msg_add("Masina e Pornita!", String(beforebat));
          delay(100);                       //prevent car contact on/off spamm
        }
      } else {
        if (contact != false) { //if (contact != false) {
          contact_count++;
          if (contact_count >= 15) {
            contact_count = 0;
            DEBUG_PRINT("~~~~Contact Off!");
            idlecount = 0;
            contact = false;
            lcd_msg_add("Masina e stinsa!", String((beforebat - check_battery()) * (-1)));
            beforebat = check_battery();             //save the voltage of the car again
            delay(100);                       //prevent car contact on/off spamm
          }
        }
      }
    }



    ///////////////////// RADAR STATUS READ
    if (digitalRead(alarmTriggerPin) == HIGH) {
      if (radarTrigger != true) {
        DEBUG_PRINT("~~~~Radar triggered!");
        radarTrigger = true;
      }
    }
    else {
      if (radarTrigger != false) {
        DEBUG_PRINT("~~~~Radar OFF!");
        radarTrigger = false;
      }
    }

    /////////////// print lcd msgs from querty
    lcd_msg();

    /////////////// switch to normal signals mode or USA STYLE
    signalswatch();

    ///////////////// light sensor reading
    readlightsensor();

    /////////////////// automatic light
    autoparkinglight();

    ///////////////////////// automatic low beam
    autolowbeam();

    ///////////////////////// automatic foot lights//interior lights
    autofootlights();

  }

  //program start
  if (millis() - last_millis_s >= 150) {
    last_millis_s = millis();

    //BEGIN HOMEPAGES !~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if (sleepStatus == 0) { //run only in awake
      if (lcdf == 0) { //run only if the lcd clear function is allowing you
        if (menumode <= 0) { // run only if you are in the main menu <=0
          if (lcdstatus == 1) {  // run only if the screen is ON
            if (menumode == 0) { //Homepage 0 -- temperature + lock/unlock
              lcd_on();
              // 16 ch per line
              lcd.setCursor(0, 0);
              if ((idlecount % 10) == 1) { //30 = 50*30 = 1500ms + other delays
                int chk = DHT.read11(tempPin);
                int tempC_temp = DHT.temperature;
                if (abs(tempC_temp) <= 60) { // anti-bad values
                  tempC_full = tempC_full + 1;
                  tempC = tempC + tempC_temp;
                  if (tempC_full == 4) {
                    tempC = (tempC / 4);
                    tempC_print = tempC;
                    tempC = tempC_temp;
                    tempC_full = 1;
                    DEBUG_PRINT("Read Temperature");
                  }
                }
              }
              lcd.print(tempC_print);
              lcd.print((char)223);
              lcd.print("C in masina   ");

              // 16 ch per line : LedsX RFX AlarmX
              lcd.setCursor(0, 1);
              lcd.print("NB");
              lcd.print(need_beam);
              lcd.print(" PB");
              lcd.print(ledp_status);
              lcd.print(" LB");
              lcd.print(ledl_status);
              lcd.print(" US");
              lcd.print(ledu_status);

            } //===================END menumode 0 ( homepage 1 )


            //Homepage 1 -- status
            if (menumode == -1) {
              lcd_on();
              lcd.setCursor(0, 0);
              lcd.print("Bat: ");
              lcd.print(map((check_battery() * 10), 105, 127, 0, 100));
              lcd.print(" % (");
              lcd.print(check_battery());
              lcd.print("v)  ");

              // 16 ch per line :LockX6 UsaX S400
              lcd.setCursor(0, 1);
              ////////////////////////////////////
              lcd.print("Lock");
              if (doorlock == false) {
                lcd.write(8); //car is locked
              }
              else {
                lcd.write(7);  //car NOT is locked
              }

              ///////////////////////////////////
              lcd.print(" Usa");
              if (door == false) {
                lcd.write(8);
              }
              else {
                lcd.write(7);
              }
              lcd.print(" Car");
              if (contact == false) {
                lcd.write(8);
              }
              else {
                lcd.write(7);
              }
              lcd.write("        ");


              ////////////////////////////////////
            } // end homepage 1




            //Homepage 2 -- LED color
            if (menumode == -2) {
              lcd_on();
              lcd.setCursor(0, 0);
              lcd.print("<Culoarea Banda>    ");
              lcd.setCursor(0, 1);
              lcd.print("R: ");
              lcd.print(redVal);
              lcd.print(" G: ");
              lcd.print(grnVal);
              lcd.print(" B: ");
              lcd.print(bluVal);
              lcd.print("   ");

              if (digitalRead(buttonPin) == LOW) {
                menumode = -22;
                click_sound(); // --- buzzer sound
                delay(400);
              }
            }
            if (menumode == -22) {
              lcd.setCursor(0, 1);

              lcd.print("R: ");
              lcd.print(redVal);
              lcd.print(" G: ");
              lcd.print(grnVal);
              lcd.print(" B: ");
              lcd.print(bluVal);
              lcd.print("   ");

              // -----------------------------------------BEGIN COLOR MIXER
              //--------------------------------------------------------
              potVal = analogRead(potPin);   // read the potentiometer value at the input pin

              if (potVal < 341)  // Lowest third of the potentiometer's range (0-340)
              {
                potVal = (potVal * 3) / 4; // Normalize to 0-255

                redVal = 256 - potVal;  // Red from full to off
                grnVal = potVal;        // Green from off to full
                bluVal = 1;             // Blue off
              }
              else if (potVal < 682) // Middle third of potentiometer's range (341-681)
              {
                potVal = ( (potVal - 341) * 3) / 4; // Normalize to 0-255

                redVal = 1;            // Red off
                grnVal = 256 - potVal; // Green from full to off
                bluVal = potVal;       // Blue from off to full
              }
              else  // Upper third of potentiometer"s range (682 - 1023)
              {
                potVal = ( (potVal - 683) * 3) / 4; // Normalize to 0-255

                redVal = potVal;       // Red from off to full
                grnVal = 1;            // Green off
                bluVal = 256 - potVal; // Blue from full to off
              }

              analogWrite(ledRPin, (redVal - ((redVal * ledlight) / 100)));   // Write values to LED pins
              analogWrite(ledGPin, (grnVal - ((grnVal * ledlight) / 100)));
              analogWrite(ledBPin, (bluVal - ((bluVal * ledlight) / 100)));
              delay(5);
              //--------------------------------------------------------
              //------------------------------------------ END COLOR MIXER
              if (digitalRead(buttonPin) == LOW) {
                EEPROM.write(0, redVal); //write to memory
                EEPROM.write(1, grnVal);
                EEPROM.write(2, bluVal);
                idlecount = 0;
                menumode = 0;
                delay(300);
                click_sound(); // --- buzzer sound
              }
            } //end homepage 2


            //Homepage 3 -- sleep it
            if (menumode == -3) {
              lcd_on();
              lcd.setCursor(0, 0);
              lcd.print("<Adormire>         ");
              lcd.setCursor(0, 1);
              lcd.print("<OK>             ");
              if (button == false) {
                sleepStatus = 1;
              }
            } // end homepage 3

            //Homepage 4 -- Open settings
            if (menumode == -4) {
              lcd_on();
              lcd.setCursor(0, 0);
              lcd.print("<Setari>           ");
              lcd.setCursor(0, 1);
              lcd.print("<deschide>          ");
              if (button == false) {
                //OPEN settings
                click_sound(); // --- buzzer sound
                menumode = 1;
                idlecount = 0;
                delay(400);
              }
            } // end homepage 4

          }                                  //end if (lcdstatus = 1)



          ///////////////// KEEP THE MENU WITH THE POTENTIOMETER
          if (menumode > -5) {               // if you are in the main MENU
            if (menumode <= 0) {             // if you are in the main MENU
              int pot = map(analogRead(potPin), 0, 1000, 0, -4);  // number of settings submenus // to EDIT ~!~~~~~~~~~~~~~
              menusel = pot;                 //read the selection from the pot
              if (menumode != menusel) {     //only if is not already set
                short_sound();
                menumode = menusel;
                lcd_on();
                idlecount = 0;
                DEBUG_PRINT("Adj POT");
              }
            }                                 // if you are in the main MENU
          }                                   // if you are in the main MENU



          //////////////////////// LCD SLEEP function ~~~~~~~~~~~
          idlecount = idlecount + 1;          //reset counter // timer

          if (idlecount >= 400) {     //27s - 300 counters
            if (menumode != -11) {    //exception
              if (lcdstatus != 0) {
                lcd_off();              //turn the lcd off
              }
            }
          }

          ////////////////////// GO SLEEP FUNCTION ~~~~~~~~~~~~~~
          if (idlecount >= 500) {     // 49 delay * 2000 = 98000ms = ~98 sec
            if (contact == false) {    // sleep arduino after 45s if the car is off
              if (door == true) {      // if the door is closed (high)
                sleepStatus = 1;     //set the flag for the sleepmode
              }
            }
          }

          //////////////////////// idlecounter protection
          if (idlecount >= 20000) {   //if the idlecount reach the limit for no reason
            idlecount = 0;            //auto reset if bugged
          }



        }  //END HOMEPAGES

      } // end lcd clear

    } //sleep mode end


    if (menumode <= 0) { //if not in settings !

      ////////////////////////////// TOGGLERS TOGGLERS TOGGLERS~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      if (sleepStatus == 0) { // if not sleeping

        // BEGIN LED STRIP toggler
        if (digitalRead(ledPin) == HIGH) { //if toggler is ON
          if (ledToggler == 0) {
            click_sound();
            idlecount = 0;
            ledToggler = 1;
            DEBUG_PRINT("LED TOGGLER ON");
            lcd_msg_add("FOOTLIGHT " + String((100 - ledlight)) + "%", "USA LIGHTS " + String(usalight) + "%");
            ledi_status = 2; //turn on the footlights animation
            ledi_timeout = 5000;
            menumode = 0;
            idlecount = 0;
            delay(100);
          }
        }
        else //if toggler is off
        {
          if (ledToggler == 1) {
            click_sound();
            idlecount = 0;
            ledToggler = 0;
            DEBUG_PRINT("LED TOGGLER OFF");
            lcd_msg_add("Banda LED OFF", "USA LIGHTS OFF");
            ledi_status = 1;
            menumode = 0;
            idlecount = 0;
            delay(100);
          }
        }
        // END LED STRIP toggler




        // BEGIN ALARM TOGGLER watchdog
        if (digitalRead(alarmPin) == HIGH) { //if the alarm is enabled
          if (alarmToggler == 0) {
            click_sound();
            idlecount = 0;
            alarmToggler = 1;
            DEBUG_PRINT("ALARM TOGGLER ON");
            lcd_msg_add("Alarm is ON", "");
            menumode = 0;
            idlecount = 0;
            delay(100);
          }
        }
        else { //if the button is LOW
          if (alarmToggler == 1) {
            click_sound();
            idlecount = 0;
            alarmToggler = 0;
            DEBUG_PRINT("LED TOGGLER OFF");
            lcd_msg_add("Alarm is OFF", "");

            menumode = 0;
            idlecount = 0;
            delay(100);
          }
        }
        // END AlARM TOGGLER feedback




        // BEGIN RF unlocking watchdog
        if (digitalRead(AutoLPin) == HIGH) {
          if (autoLightStatus == 0) {
            click_sound();
            idlecount = 0;
            autoLightStatus = 1;
            DEBUG_PRINT("AutoLighing TOGGLER ON");
            lcd_msg_add("AutoLighing ON!", "");
            menumode = 0;
            idlecount = 0;
            delay(100);
          }
        }
        else { //if the button is LOW
          if (autoLightStatus == 1) {
            click_sound();
            idlecount = 0;
            autoLightStatus = 0;
            DEBUG_PRINT("AutoLighing TOGGLER OFF");
            lcd_msg_add("AutoLighing OFF!", "");
            menumode = 0;
            idlecount = 0;
            delay(100);
          }
        }

      }     //end togglers
    }       // end sleep mode


    ///////////////////////////////////////////////SETTINGS MODE ! ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ SETTINGS
    if (menumode >= 1) {
      if (menumode == 1) { //welcome message
        lcd.clear();
        lcd.setCursor(5, 0);
        lcd.write("SETARI");
        lcd.setCursor(4, 1);
        lcd.write("avansate");

        delay(1000);
        menumode = 2;
        menupage = 0;
        lcd.clear();
        click_sound(); // --- buzzer sound

        count = 0;
      }
      else { //menu

        //begin setting the selection
        int pot = map(analogRead(potPin), 0, 1025, 1, 7);  // 7 = number of settings submenus
        menusel = pot; //read the selection from the pot

        lcd.setCursor(0, 0);
        lcd.print(" ");

        if (menupage <= 2) { //set it only if your out of a submenu

          lcd.setCursor(0, 1);
          lcd.print(" ");
          if ( (pot % 2) == 0) {
            lcd.setCursor(0, 1);
            lcd.print("~");
            if (settingspos == 0) {
              settingspos = 1;
              click_sound(); // --- buzzer sound
            }
          }
          else {
            lcd.setCursor(0, 0);
            lcd.print("~");
            if (settingspos == 1) {
              settingspos = 0;
              click_sound(); // --- buzzer sound
            }
          }

          //end of setting the selection

          //page switcher
          if (menusel >= 1 and menusel <= 2) {
            menupage = 0;
          }
          if (menusel >= 3 and menusel <= 4) {
            menupage = 1;
          }
          if (menusel >= 5 and menusel <= 6) {
            menupage = 2;
          }
        }


        if (menupage == 0) {
          lcd.setCursor(1, 0);
          lcd.print("1.FOOT Lights   ");
          lcd.setCursor(1, 1);
          lcd.print("2.US Lights       ");
        }
        if (menupage == 1) {
          lcd.setCursor(1, 0);
          lcd.print("3.Alarma       ");
          lcd.setCursor(1, 1);
          lcd.print("4.Infos   ");
        }
        if (menupage == 2) {
          lcd.setCursor(1, 0);
          lcd.print("5.Infos2        ");
          lcd.setCursor(1, 1);
          lcd.print("6.Iesi         ");
        }
        //LED STRIP MENU
        if (menupage >= 3) {
          if (menupage == 3) {
            lcd.setCursor(0, 0);
            lcd.print("Inainte: Dupa:");
            lbg1.drawValue( ledlight, 100);
            lbg.drawValue( analogRead(potPin), 1024);
            lcd.setCursor(7, 1); //mid line
            lcd.print("[]");
            menudone = 1;
          }
          //USA LIGHTS MENU
          if (menupage == 4) {
            lcd.setCursor(0, 0);
            lcd.print("Inainte: Dupa:");
            lbg1.drawValue( usalight, 100);
            lbg.drawValue( analogRead(potPin), 1024);
            lcd.setCursor(7, 1); //mid line
            lcd.print("[]");
            menudone = 2;
          }

          //ALARM MENU
          if (menupage == 5) {
            lcd.setCursor(0, 0);
            lcd.print("Inainte: Dupa:");
            lcd.setCursor(0, 1); //mid line
            lcd.print(alarmtime);
            lcd.print("/30s");
            lcd.setCursor(9, 1); //mid line
            lcd.print(map(analogRead(potPin), 0, 1025, 1, 30));
            lcd.print("/30s ");
            lcd.setCursor(7, 1); //mid line
            lcd.print("[]");
            menudone = 3;
          }

          //inchidere
          if (menupage == 6) {
            int temp00 = map(analogRead(potPin), 0, 1025, 1, 6);  // 4 = number of settings submenus
            switch (temp00) {
              case 1:
                lcd.setCursor(0, 0);
                lcd.print("DoorlockPin:     ");
                lcd.setCursor(0, 1); //mid line
                lcd.print(digitalRead(doorlockPin));
                lcd.print("  ");
                break;

              case 2:
                lcd.setCursor(0, 0);
                lcd.print("DoorLock:         ");
                lcd.setCursor(0, 1); //mid line
                lcd.print(doorlock);
                lcd.print("  ");
                break;

              case 3:
                lcd.setCursor(0, 0);
                lcd.print("Alarm togger         ");
                lcd.setCursor(0, 1); //mid line
                lcd.print(digitalRead(alarmPin));
                lcd.print("  ");
                break;

              case 4:
                lcd.setCursor(0, 0);
                lcd.print("Alarm trigger      ");
                lcd.setCursor(0, 1); //mid line
                lcd.print(radarTrigger);
                lcd.print("  ");
                break;

              case 5:
                lcd.setCursor(0, 0);
                lcd.print("Alarm CD        ");
                lcd.setCursor(0, 1); //mid line
                lcd.print(alarmCD / 100);
                lcd.print("  ");
                break;
            }
            //end~~~~~~
            menudone = 4;
            delay(5); //visualization fixer
          }

          //EMPTY menu
          if (menupage == 7) {
            lcd.setCursor(0, 0);
            lcd.print("~~~");
            lcd.setCursor(0, 1);
            lcd.print("~~~");
            menudone = 5;
          }
          //exit
          if (menupage == 8) {
            menumode = 0;
          }

        }

        if (button == false) {
          click_sound(); // --- buzzer sound

          //LCD STRIP MENU
          if (menupage >= 3) { //back to menu from led strip settings
            delay(150);
            lcd.clear();
            lcd.setCursor(0, 0);
            if (menudone == 1) {
              ledlight = map(analogRead(potPin), 0, 1025, 0, 100); //set the new value
              lcd.print("Led opacity: ");
              lcd.print(ledlight);
              lcd.print(" % ");
              EEPROM.write(9, ledlight);
            }
            if (menudone == 2) {
              usalight = map(analogRead(potPin), 0, 1025, 0, 100); //set the new value
              lcd.print("USA opacity: ");
              lcd.print(usalight);
              lcd.print(" % ");
              EEPROM.write(8, usalight);
            }
            if (menudone == 3) {
              alarmtime = map(analogRead(potPin), 0, 1025, 1, 30); //set the new value
              lcd.print("Alarm time: ");
              lcd.print(alarmtime);
              lcd.print("s");
              EEPROM.write(5, alarmtime);
            }
            if (menudone == 4) {
              lcd.print("~~~~4");
            }
            if (menudone == 5) {
              lcd.print("~~~~5");
            }
            menupage = 1;
            menumode = 2; // no click
            click_sound(); // --- buzzer sound
            delay(1500);
            lcd.clear();
          }

          else if (menupage <= 2) { //ENTERING A MENU
            if (menumode != 3) {
              if (menusel == 6) {
                menumode = 0;
                click_sound(); // --- buzzer sound
              }

              delay(150);

              menupage = (menusel + 2);
              click_sound(); // --- buzzer sound
              lcd.clear();
            }
          }
        }

        //end of timeout
      }
    } // end settings

  }

  if (sleepStatus == 1) {
    go_sleep(); // go to sleep at the end of the loop
  }
} //end millis all
