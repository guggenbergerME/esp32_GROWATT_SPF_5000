/*

MAX                         ESP
DI (Driver Input)           19 tx
DE (Driver Enable Input)    4
RE (Receiver Enable Input)  4
RO (Receiver Output)        18 rx


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
    uint16_t reg = 0x03;
    int result;
    switch (actual)
    {
      case 0 :
        reg = 0x03;
        result = serverA.readInputRegisters(reg, 8);
        if (result == serverA.ku8MBSuccess) // do something if read is successfull
        {
          Serial.print(reg, HEX); Serial.println("********************************************");
          Serial.print(F("Charging equipment input V:     ")); Serial.println(serverA.getResponseBuffer(0x00) / 100.0f);
          Serial.print(F("Charging equipment input A:     ")); Serial.println(serverA.getResponseBuffer(0x01) / 100.0f);
          Serial.print(F("Charging equipment input W:     ")); Serial.println((serverA.getResponseBuffer(0x02) + ((uint32_t)serverA.getResponseBuffer(0x03) << 16)) / 100.0f); // cast and brackets
          Serial.print(F("Charging equipment output V:    ")); Serial.println(serverA.getResponseBuffer(0x04) / 100.0f);
          Serial.print(F("Charging equipment output A:    ")); Serial.println(serverA.getResponseBuffer(0x05) / 100.0f);
          Serial.print(F("Charging equipment output W:    ")); Serial.println((serverA.getResponseBuffer(0x06) + ((uint32_t)serverA.getResponseBuffer(0x07) << 16)) / 100.0f); // cast and brackets
        }
        else
        {
          Serial.print(F(" ServerA no success register ")); Serial.print(reg, HEX); Serial.print(F(" result=")); Serial.println(result, HEX);
        }
        actual++;
        break;

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

      case 2 :
        reg = 0x032;
        result = serverA.readInputRegisters(reg, 7);
        if (result == serverA.ku8MBSuccess) // do something if read is successfull
        {
          Serial.print(reg, HEX); Serial.println("********************************************");
          Serial.print(F("Charging equipment input V:     ")); Serial.println(serverA.getResponseBuffer(0x00 - reg) / 100.0f);
          Serial.print(F("Charging equipment input A:     ")); Serial.println(serverA.getResponseBuffer(0x01 - reg) / 100.0f);
          Serial.print(F("Charging equipment input W:     ")); Serial.println((serverA.getResponseBuffer(0x02 - reg) + ((uint32_t)serverA.getResponseBuffer(0x310F - reg) << 16)) / 100.0f); // cast and brackets
          Serial.print(F("Battery Temperature:            ")); Serial.println(serverA.getResponseBuffer(0x03 - reg) / 100.0f);
          Serial.print(F("Temperature inside equipment:   ")); Serial.println(serverA.getResponseBuffer(0x04 - reg) / 100.0f);
          Serial.print(F("Power components temperature:   ")); Serial.println(serverA.getResponseBuffer(0x05 - reg) / 100.0f);
        }
        else
        {
          Serial.print(F(" ServerA no success register ")); Serial.print(reg, HEX); Serial.print(F(" result=")); Serial.println(result, HEX);
        }
        actual++;
        break;


          default : actual = 0;
    } // switch
  }
}


// Parameters don't change so often, we only read them once after startup
// preTransmission and postTransmission take care of the requested "delay" between requests
void requestParameter()
{
  uint16_t reg;
  int result;

  reg = 0x3000;
  result = serverA.readInputRegisters(reg, 8);
  if (result == serverA.ku8MBSuccess) // do something if read is successfull
  {
    Serial.print(F("Charging equipment rated input V: ")); Serial.println(serverA.getResponseBuffer(0x00) / 100.0f);
    Serial.print(F("Charging equipment rated input A: ")); Serial.println(serverA.getResponseBuffer(0x01) / 100.0f);
    Serial.print(F("Charging equipment rated input W: ")); Serial.println((serverA.getResponseBuffer(0x02) + ((uint32_t)serverA.getResponseBuffer(0x03) << 16)) / 100.0f); // cast and brackets
    Serial.print(F("Charging equipment rated output V:")); Serial.println(serverA.getResponseBuffer(0x04) / 100.0f);
    Serial.print(F("Charging equipment rated output A:")); Serial.println(serverA.getResponseBuffer(0x05) / 100.0f);
    Serial.print(F("Charging equipment rated output W:")); Serial.println((serverA.getResponseBuffer(0x06) + ((uint32_t)serverA.getResponseBuffer(0x07) << 16)) / 100.0f); // cast and brackets
    Serial.print(F("Charging mode:                    ")); Serial.println(serverA.getResponseBuffer(0x08));
  }
  else
  {
    Serial.print(F(" ServerA no success register ")); Serial.print(reg, HEX); Serial.print(F(" result=")); Serial.println(result, HEX);
  }
  
  reg = 0x2;
  result = serverA.readCoils(reg, 1);
  if (result == serverA.ku8MBSuccess)
  {
    Serial.print(F("Manual control the load:        ")); Serial.println(serverA.getResponseBuffer(0x2 - reg));
  }
  else
  {
    Serial.print(F(" ServerA no success register ")); Serial.print(reg, HEX); Serial.print(F(" result=")); Serial.println(result, HEX);
  }

  reg = 0x5;
  result = serverA.readCoils(reg, 2);
  if (result == serverA.ku8MBSuccess)
  {
    Serial.print(F("Enable load test mode:          ")); Serial.println(serverA.getResponseBuffer(0x5 - reg));
    Serial.print(F("Force the load on/off:          ")); Serial.println(serverA.getResponseBuffer(0x6 - reg));
  }
  else
  {
    Serial.print(F(" ServerA no success register ")); Serial.print(reg, HEX); Serial.print(F(" result=")); Serial.println(result, HEX);
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
  //requestParameter();
}

void loop()
{
  requestData();
}