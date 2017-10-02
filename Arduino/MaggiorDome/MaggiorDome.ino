
/**************************************************************************/
/*!
  Firmware: MaggiorDome
  Version: 0.1.2
  Scope: Utility control for astronomical dome
  Author: Simone Martina
  
  Usage:
  Relay | ON | OFF |   Comment
  -------------------------------------------
  K1    | -  |  -  | Case fans
  K2    | A  |  B  | Observatory red light
  K3    | C  |  D  | N/A
  K4    | E  |  F  | N/A
  K5    | G  |  H  | N/A
  K6    | I  |  J  | N/A
  K7    | K  |  L  | 12V (mount, focuser...)
  K8    | M  |  N  | 220V (CCD)
  -------------------------------------------
  
  Changelog:
  2017/07/12 - v0.1 Case sensor, case fan controls
  2017/07/27 - v0.1.1 Mount & CCD serial command
  2017/07/28 - v0.1.2 Added EEPROM support to save mount, ccd and light last state
  2017/08/05 - v0.1.3 Added an external step relay on K8 due to not overload K8 array relay
               Added: k8 pulse relay, save last state, avoid double commands
*/
/**************************************************************************/

#include <Wire.h>
#include "Adafruit_MCP9808.h"
#include <EEPROM.h>

/**** USER PARAMETERS ****/

// Temperature check interval in ms
const long tempInt = 300000;

// Case temperature limit
const float caseLimit = 40;

/*****************************************************************/

// Relays pins
const int k1=12;
const int k2=11;
const int k3=10;
const int k4=9;
const int k5=8;
const int k6=7;
const int k7=6;
const int k8=5;

// Overflow workaround
unsigned long previousMillis=0;

// Char variable for serial commands
char cmd;

// Create the case temperature sensor object
Adafruit_MCP9808 caseSensor = Adafruit_MCP9808();

void setup() {
  Serial.begin(9600);
  Serial.println("MaggiorDome 0.1.3");

  // Relay pins initialization
  // Case FANs:
  pinMode(k1,OUTPUT);
  // Observatory red light:
  pinMode(k2,OUTPUT);
  pinMode(k3,OUTPUT);
  pinMode(k4,OUTPUT);
  pinMode(k5,OUTPUT);
  pinMode(k6,OUTPUT);
  // Case cap connection (mount, focuser, dew heater):
  pinMode(k7,OUTPUT);
  // CCD 220V:
  pinMode(k8,OUTPUT);

  // Turn ON case fan during startup
  digitalWrite(k1,LOW);
  Serial.println("Case fan ON");

  if ( EEPROM.read(k2) == 1 ) {
    digitalWrite(k2, LOW);
    Serial.println("Recover: Observatory red light ON");
  } else {
    digitalWrite(k2,HIGH);
  }
  digitalWrite(k3,HIGH);
  digitalWrite(k4,HIGH);
  digitalWrite(k5,HIGH);
  digitalWrite(k6,HIGH);
  digitalWrite(k7,HIGH);
  if ( EEPROM.read(k7) == 1 ) {
    Serial.println("Recover: Mount ON");
  }
  digitalWrite(k8,HIGH);
  if ( EEPROM.read(k8) == 1 ) {
    Serial.println("Recover: CCD was ON");
  }
  
  // Make sure the sensor is found, you can also pass in a different i2c
  // address with tempsensor.begin(0x18) for example
  if (!caseSensor.begin(0x18)) {
    Serial.println("Couldn't find case temperature sensor!");
  }
}

void loop() {
  // Wait for command only if serial is connected
  if(Serial.available() > 0 ) {
    cmd = Serial.read();
    
    if ( cmd == 'A' ) {
        digitalWrite(k2, LOW);
        EEPROM.write(k2, 1);
        Serial.println("Observatory red light ON");
    } else if ( cmd == 'B' ) {
        digitalWrite(k2, HIGH);
        EEPROM.write(k2, 0);
        Serial.println("Observatory red light OFF");
    } else if ( cmd == 'C' ) {
        digitalWrite(k3, LOW);
        EEPROM.write(k3, 1);
        Serial.println("k3 relay activated");
    } else if ( cmd == 'D' ) {
        digitalWrite(k3, HIGH);
        EEPROM.write(k3, 0);
        Serial.println("k3 relay deactivated");
    } else if ( cmd == 'E' ) {
        digitalWrite(k4, LOW);
        EEPROM.write(k4, 1);
        Serial.println("k4 relay activated");
    } else if ( cmd == 'F' ) {
        digitalWrite(k4, HIGH);
        EEPROM.write(k4, 0);
        Serial.println("k4 relay deactivated");
    } else if ( cmd == 'G' ) {
        digitalWrite(k5, LOW);
        EEPROM.write(k5, 1);
        Serial.println("k5 relay activated");
    } else if ( cmd == 'H' ) {
        digitalWrite(k5, HIGH);
        EEPROM.write(k5, 0);
        Serial.println("k5 relay deactivated");
    } else if ( cmd == 'I' ) {
        digitalWrite(k6, LOW);
        EEPROM.write(k6, 1);
        Serial.println("k4 relay activated");
    } else if ( cmd == 'J' ) {
        digitalWrite(k6, HIGH);
        EEPROM.write(k6, 0);
        Serial.println("k4 relay deactivated");
    } else if ( cmd == 'K' ) {
      if ( EEPROM.read(k7) == 0 ) {
          digitalWrite(k7, LOW);
          delay(250);
          digitalWrite(k7, HIGH);
          EEPROM.write(k7, 1);
          Serial.println("Munt ON");
        } else {
          Serial.println("Warning: Mount was just ON");
        }
    } else if ( cmd == 'L' ) {
      if ( EEPROM.read(k7) == 1 ) {
          digitalWrite(k7, LOW);
          delay(250);
          digitalWrite(k7, HIGH);
          EEPROM.write(k7, 0);
          Serial.println("Mount OFF");
      } else {
        Serial.println("Warning: Mount was just OFF");
      }
    } else if ( cmd == 'M' ) {
        if ( EEPROM.read(k8) == 0 ) {
          digitalWrite(k8, LOW);
          delay(250);
          digitalWrite(k8, HIGH);
          EEPROM.write(k8, 1);
          Serial.println("CCD ON");
        } else {
          Serial.println("Warning: CCD was just ON");
        }
    } else if ( cmd == 'N' ) {
      if ( EEPROM.read(k8) == 1 ) {
          digitalWrite(k8, LOW);
          delay(250);
          digitalWrite(k8, HIGH);
          EEPROM.write(k8, 0);
          Serial.println("CCD OFF");
      } else {
        Serial.println("Warning: CCD was just OFF");
      }
    }
    
    Serial.flush();
  }
  
  // Get snapshot of time
  unsigned long currentMillis = millis();
  
  // Read the temperature of the case
  float caseTemp = caseSensor.readTempC();

  if ((unsigned long)(currentMillis - previousMillis) >= tempInt) {

    // Print temprature
    Serial.print("Case temperature: "); Serial.print(caseTemp); Serial.print("*C\t");
    previousMillis = currentMillis;

    if (caseTemp > caseLimit) {
      // Turn ON k1 relay
      digitalWrite(k1,LOW);
      Serial.println("Case fan ON"); 
    }
    else
    {
      // Turn OFF k1 relay
      digitalWrite(k1,HIGH);
      Serial.println("Case fan OFF");
    }
  }
  
  delay(1000);
}
