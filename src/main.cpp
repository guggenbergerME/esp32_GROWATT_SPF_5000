/*

MAX                         ESP
DI (Driver Input)           19 tx
DE (Driver Enable Input)    4
RE (Receiver Enable Input)  4
RO (Receiver Output)        18 rx


Register Anzeigen

Die Adresse des ersten Registers (40108-40001 = 107 = 6B hex) - > reg = 0x006B;
Die Anzahl X der erforderlichen Register (lesen 3 Register von 40108 bis 40110)
-> serverA.readInputRegisters(reg, X);

Holding Reg - readHoldingRegisters

*/
#include <Arduino.h>
/*
   Serial Interface
 * **************************************************** */
// if you don't have enough HW Serial (i.e. on an UNO)
// you are forced to use SoftwareSerial or AltSoftSerial
#include <SoftwareSerial.h>
constexpr uint8_t rxPin = 18;                   // for Softserial
constexpr uint8_t txPin = 19;
SoftwareSerial mySerial(rxPin, txPin);


/* **************************************************** *
   Modbus
 * **************************************************** */
#include <ModbusMaster.h>                        // Modbus Master 2.0.0 by Doc Walker - install with Library Manager
constexpr uint8_t modbusEnablePin = 4;           // The GPIO used to control the MAX485 TX pin. Set to 255 if you are not using RS485 or a selfsensing adapter
constexpr uint32_t modbusBaud = 9600;          // use slow speeds with SoftSerial
constexpr uint16_t modbusRestTx = 15000;         // rest time between transmissions - microseconds
uint32_t modbusPreviousTx = 0;                   // timestamp of last transmission - microseconds
ModbusMaster serverA;                            // instantiate ModbusMaster object - slave - node

/// this function will be called before the client transmits data
void preTransmission()
{
  while (micros() - modbusPreviousTx < modbusRestTx)   // check last transmission end and wait if the call was to early
  {
    yield();                                           // wait some time and allow background tasks
  }
  digitalWrite(modbusEnablePin, HIGH);
}

// this function will be called after the transmission
void postTransmission()
{
  digitalWrite(modbusEnablePin, LOW);
  modbusPreviousTx = micros();         // remember last timestamp
}

// do all the settings for the Modbus
void modbusInit()
{
  mySerial.begin(modbusBaud);                     // initialize Modbus communication baud rate
  serverA.begin(1, mySerial);                     // communicate with Modbus server ID over the given Serial interface
  pinMode(modbusEnablePin, OUTPUT);               // Init enable pins for modbus master library
  digitalWrite(modbusEnablePin, LOW);
  serverA.preTransmission(preTransmission);       // Callbacks allow us to configure the RS485 transceiver correctly
  serverA.postTransmission(postTransmission);
}

/* **************************************************** *
   Modbus - Implementation
 * **************************************************** */

void requestData()
{
  static uint32_t previousMillis = 5000;  // timestamp of last request
  static uint8_t actual = 0;           // actual iteration
  uint32_t currentMillis = millis();

  if (currentMillis - previousMillis > 5000)  // set the interval in ms
  {
    previousMillis = currentMillis;
    uint16_t reg;
    int result;
    switch (actual)
    {
      case 0 :
        reg = 0x0015;
        result = serverA.readHoldingRegisters(reg, 16);
        if (result == serverA.ku8MBSuccess) // do something if read is successfull
        {
          Serial.println(F(" "));
          Serial.print(F("1 - ")); Serial.print(serverA.getResponseBuffer(0x00), HEX);Serial.print(" -> ");Serial.println(serverA.getResponseBuffer(0x00));
          Serial.print(F("2 - ")); Serial.print(serverA.getResponseBuffer(0x01), HEX);Serial.print(" -> ");Serial.println(serverA.getResponseBuffer(0x01));
          Serial.print(F("3 - ")); Serial.print(serverA.getResponseBuffer(0x02), HEX);Serial.print(" -> ");Serial.println(serverA.getResponseBuffer(0x02));
          Serial.print(F("4 - ")); Serial.print(serverA.getResponseBuffer(0x03), HEX);Serial.print(" -> ");Serial.println(serverA.getResponseBuffer(0x03));
          Serial.print(F("5 - ")); Serial.print(serverA.getResponseBuffer(0x04), HEX);Serial.print(" -> ");Serial.println(serverA.getResponseBuffer(0x04));
          Serial.print(F("6 - ")); Serial.print(serverA.getResponseBuffer(0x05), HEX);Serial.print(" -> ");Serial.println(serverA.getResponseBuffer(0x05));
          Serial.print(F("7 - ")); Serial.print(serverA.getResponseBuffer(0x06), HEX);Serial.print(" -> ");Serial.println(serverA.getResponseBuffer(0x06));
          Serial.print(F("8 - ")); Serial.print(serverA.getResponseBuffer(0x07), HEX);Serial.print(" -> ");Serial.println(serverA.getResponseBuffer(0x07));
          
          Serial.print(F("9 - ")); Serial.print(serverA.getResponseBuffer(0x08), HEX);Serial.print(" -> ");Serial.println(serverA.getResponseBuffer(0x08));
          Serial.print(F("10- ")); Serial.print(serverA.getResponseBuffer(0x09), HEX);Serial.print(" -> ");Serial.println(serverA.getResponseBuffer(0x09));
          Serial.print(F("11- ")); Serial.print(serverA.getResponseBuffer(0x0A), HEX);Serial.print(" -> ");Serial.println(serverA.getResponseBuffer(0x0A));
          Serial.print(F("12- ")); Serial.print(serverA.getResponseBuffer(0x0B), HEX);Serial.print(" -> ");Serial.println(serverA.getResponseBuffer(0x0B));
          Serial.print(F("13- ")); Serial.print(serverA.getResponseBuffer(0x0C), HEX);Serial.print(" -> ");Serial.println(serverA.getResponseBuffer(0x0C));
          Serial.print(F("14- ")); Serial.print(serverA.getResponseBuffer(0x0D), HEX);Serial.print(" -> ");Serial.println(serverA.getResponseBuffer(0x0D));
          Serial.print(F("15- ")); Serial.print(serverA.getResponseBuffer(0x0E), HEX);Serial.print(" -> ");Serial.println(serverA.getResponseBuffer(0x0E));
          Serial.print(F("16- ")); Serial.print(serverA.getResponseBuffer(0x0F), HEX);Serial.print(" -> ");Serial.println(serverA.getResponseBuffer(0x0F));
          
        }
        else
        {
          Serial.print(F(" ServerA no success register ")); Serial.print(reg, HEX); Serial.print(F(" result=")); Serial.println(result, HEX);
        }
        actual++;
        break;
/*
      case 1 :
        reg = 0x031;
        result = serverA.readInputRegisters(reg, 7);
        if (result == serverA.ku8MBSuccess) // do something if read is successfull
        {
          Serial.print(reg, HEX); Serial.println("********************************************");
          Serial.print(F("Charging equipment input V:     ")); Serial.println(serverA.getResponseBuffer(0x310c - reg) / 100.0f);
          Serial.print(F("Charging equipment input A:     ")); Serial.println(serverA.getResponseBuffer(0x310d - reg) / 100.0f);
          Serial.print(F("Charging equipment input W:     ")); Serial.println((serverA.getResponseBuffer(0x310e - reg) + ((uint32_t)serverA.getResponseBuffer(0x310F - reg) << 16)) / 100.0f); // cast and brackets
          Serial.print(F("Battery Temperature:            ")); Serial.println(serverA.getResponseBuffer(0x3110 - reg) / 100.0f);
          Serial.print(F("Temperature inside equipment:   ")); Serial.println(serverA.getResponseBuffer(0x3111 - reg) / 100.0f);
          Serial.print(F("Power components temperature:   ")); Serial.println(serverA.getResponseBuffer(0x3112 - reg) / 100.0f);
        }
        else
        {
          Serial.print(F(" ServerA no success register ")); Serial.print(reg, HEX); Serial.print(F(" result=")); Serial.println(result, HEX);
        }
        actual++;
        break;
*/

          default : actual = 0;
    } // switch
  }
}




/* **************************************************** *
   setup and loop
 * **************************************************** */

void setup()
{
  Serial.begin(115200);
  Serial.println(F("Modbus Client Example 0x60"));

  modbusInit();
}

void loop()
{
  requestData();
}