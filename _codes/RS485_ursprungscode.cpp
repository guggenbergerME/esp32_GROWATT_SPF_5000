/*******************************************************

  Modbus Client Example 0x60 (Modbus Master)
  Read EPSOLAR - all registers

  This Modbus Client
  - reads registers from Modbus Servers periodically

  based on an idea

  RS485_HalfDuplex.pde - example using ModbusMaster library to communicate
  with EPSolar LS2024B controller using a half-duplex RS485 transceiver.

  This example is tested against an EPSolar LS2024B solar charge controller.
  See here for protocol specs:
  http://www.solar-elektro.cz/data/dokumenty/1733_modbus_protocol.pdf

  hardware
  - a LED
  - 4 push buttons
  - a MAX485-TTL adapter

  by noiasca
  2022-07-30

 *******************************************************/


/* *******************************************************
   Serial Interface
 * **************************************************** */
// if you don't have enough HW Serial (i.e. on an UNO)
// you are forced to use SoftwareSerial or AltSoftSerial
//include <SoftwareSerial.h>
//constexpr uint8_t rxPin = 2;                   // for Softserial
//constexpr uint8_t txPin = 3;
//SoftwareSerial mySerial(rxPin, txPin);

// On a Mega you can simply use
// a Reference to an existing HW Serial:
HardwareSerial &mySerial = Serial3;

/* **************************************************** *
   Modbus
 * **************************************************** */

#include <ModbusMaster.h>                        // Modbus Master 2.0.0 by Doc Walker - install with Library Manager
constexpr uint8_t modbusEnablePin = 5;           // The GPIO used to control the MAX485 TX pin. Set to 255 if you are not using RS485 or a selfsensing adapter
constexpr uint32_t modbusBaud = 115200;          // use slow speeds with SoftSerial
constexpr uint16_t modbusRestTx = 15000;         // rest time between transmissions - microseconds
uint32_t modbusPreviousTx = 0;                   // timestamp of last transmission - microseconds
ModbusMaster serverA;                            // instantiate ModbusMaster object - slave - node

// this function will be called before the client transmits data
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
  static uint32_t previousMillis = -5000;  // timestamp of last request
  static uint8_t actual = 0;           // actual iteration
  uint32_t currentMillis = millis();

  if (currentMillis - previousMillis > 5000)  // set the interval in ms
  {
    previousMillis = currentMillis;
    uint16_t reg = 0x3100;
    int result;
    switch (actual)
    {
      case 0 :
        reg = 0x3100;
        result = serverA.readInputRegisters(reg, 8);
        if (result == serverA.ku8MBSuccess) // do something if read is successfull
        {
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
        reg = 0x310C;
        result = serverA.readInputRegisters(reg, 7);
        if (result == serverA.ku8MBSuccess) // do something if read is successfull
        {
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
        reg = 0x311A;
        result = serverA.readInputRegisters(reg, 2);
        if (result == serverA.ku8MBSuccess) // do something if read is successfull
        {
          Serial.print(F("Battery SoC:                    ")); Serial.println(serverA.getResponseBuffer(0x311A - reg));
          Serial.print(F("Remote battery temperature:     ")); Serial.println(serverA.getResponseBuffer(0x311B - reg) / 100.0f);
        }
        else
        {
          Serial.print(F(" ServerA no success register ")); Serial.print(reg, HEX); Serial.print(F(" result=")); Serial.println(result, HEX);
        }
        actual++;
        break;
      case 3 :
        reg = 0x311A;
        result = serverA.readInputRegisters(reg, 1);
        if (result == serverA.ku8MBSuccess) // do something if read is successfull
        {
          Serial.print(F("Battery's real rated power:     ")); Serial.println(serverA.getResponseBuffer(0x311D - reg) / 100.0f);
        }
        else
        {
          Serial.print(F(" ServerA no success register ")); Serial.print(reg, HEX); Serial.print(F(" result=")); Serial.println(result, HEX);
        }
        actual++;
        break;
      case 4 :
        reg = 0x3200;
        result = serverA.readInputRegisters(reg, 2);
        if (result == serverA.ku8MBSuccess) // do something if read is successfull
        {
          Serial.print(F("Battery status:                 ")); Serial.println(serverA.getResponseBuffer(0x3200 - reg), BIN);
          Serial.print(F("Charging equipment:             ")); Serial.println(serverA.getResponseBuffer(0x3201 - reg), BIN);
        }
        else
        {
          Serial.print(F(" ServerA no success register ")); Serial.print(reg, HEX); Serial.print(F(" result=")); Serial.println(result, HEX);
        }
        actual++;
        break;
      case 5 :
        reg = 0x3300;
        result = serverA.readInputRegisters(reg, 30);
        if (result == serverA.ku8MBSuccess) // do something if read is successfull
        {
          Serial.print(F("Max input (PV) volt:            ")); Serial.println(serverA.getResponseBuffer(0x3300 - reg) / 100.0f);
          Serial.print(F("Min input (PV) volt:            ")); Serial.println(serverA.getResponseBuffer(0x3301 - reg) / 100.0f);
          Serial.print(F("Max battery volt:               ")); Serial.println(serverA.getResponseBuffer(0x3302 - reg) / 100.0f);
          Serial.print(F("Min battery volt:               ")); Serial.println(serverA.getResponseBuffer(0x3303 - reg) / 100.0f);
          Serial.print(F("Consumed energy today:          ")); Serial.println((serverA.getResponseBuffer(0x3304 - reg) + ((uint32_t)serverA.getResponseBuffer(0x3305 - reg) << 16)) / 100.0f);
          Serial.print(F("Consumed energy month:          ")); Serial.println((serverA.getResponseBuffer(0x3306 - reg) + ((uint32_t)serverA.getResponseBuffer(0x3307 - reg) << 16)) / 100.0f);
          Serial.print(F("Consumed energy year:           ")); Serial.println((serverA.getResponseBuffer(0x3308 - reg) + ((uint32_t)serverA.getResponseBuffer(0x3309 - reg) << 16)) / 100.0f);
          Serial.print(F("Total consumed:                 ")); Serial.println((serverA.getResponseBuffer(0x330A - reg) + ((uint32_t)serverA.getResponseBuffer(0x330B - reg) << 16)) / 100.0f);
          Serial.print(F("Generated energy today:         ")); Serial.println((serverA.getResponseBuffer(0x330C - reg) + ((uint32_t)serverA.getResponseBuffer(0x330D - reg) << 16)) / 100.0f);
          Serial.print(F("Generated energy month:         ")); Serial.println((serverA.getResponseBuffer(0x330E - reg) + ((uint32_t)serverA.getResponseBuffer(0x330F - reg) << 16)) / 100.0f);
          Serial.print(F("Generated energy year:          ")); Serial.println((serverA.getResponseBuffer(0x3310 - reg) + ((uint32_t)serverA.getResponseBuffer(0x3311 - reg) << 16)) / 100.0f);
          Serial.print(F("Total generated energy:         ")); Serial.println((serverA.getResponseBuffer(0x3312 - reg) + ((uint32_t)serverA.getResponseBuffer(0x3313 - reg) << 16)) / 100.0f);
          Serial.print(F("CO2 reduction:                  ")); Serial.println((serverA.getResponseBuffer(0x3314 - reg) + ((uint32_t)serverA.getResponseBuffer(0x3315 - reg) << 16)) / 100.0f);
          // there is one register not documented
          Serial.print(F("Battery Temperature :           ")); Serial.println(serverA.getResponseBuffer(0x331D - reg) / 100.0f);
          Serial.print(F("Ambient Temperature :           ")); Serial.println(serverA.getResponseBuffer(0x331E - reg) / 100.0f);
        }
        else
        {
          Serial.print(F(" ServerA no success register ")); Serial.print(reg, HEX); Serial.print(F(" result=")); Serial.println(result, HEX);
        }
        actual = 0; // in the last case we jump back to 0 instead of actual++;
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

  reg = 0x300E;
  result = serverA.readInputRegisters(reg, 1);
  if (result == serverA.ku8MBSuccess) // do something if read is successfull
  {
    Serial.print(F("Rated output current of load:     ")); Serial.println(serverA.getResponseBuffer(0x00) / 100.0f);
  }
  else
  {
    Serial.print(F(" ServerA no success register ")); Serial.print(reg, HEX); Serial.print(F(" result=")); Serial.println(result, HEX);
  }

  reg = 0x9000;
  result = serverA.readHoldingRegisters(reg, 15);
  if (result == serverA.ku8MBSuccess) // do something if read is successfull
  {
    Serial.print(F("Battery Type:                   ")); Serial.println(serverA.getResponseBuffer(0x9000 - reg));
    Serial.print(F("Battery Capacity:               ")); Serial.println(serverA.getResponseBuffer(0x9001 - reg));
    Serial.print(F("Temperature compensation coeff: ")); Serial.println(serverA.getResponseBuffer(0x9002 - reg) / 100.0f);
    Serial.print(F("High Volt disconnect:           ")); Serial.println(serverA.getResponseBuffer(0x9003 - reg) / 100.0f);
    Serial.print(F("Charging limit voltage:         ")); Serial.println(serverA.getResponseBuffer(0x9004 - reg) / 100.0f);
    Serial.print(F("Over voltage reconnect:         ")); Serial.println(serverA.getResponseBuffer(0x9005 - reg) / 100.0f);
    Serial.print(F("Equalization voltage:           ")); Serial.println(serverA.getResponseBuffer(0x9006 - reg) / 100.0f);
    Serial.print(F("Boost voltage:                  ")); Serial.println(serverA.getResponseBuffer(0x9007 - reg) / 100.0f);
    Serial.print(F("Float voltage:                  ")); Serial.println(serverA.getResponseBuffer(0x9008 - reg) / 100.0f);
    Serial.print(F("Boost reconnect voltage:        ")); Serial.println(serverA.getResponseBuffer(0x9009 - reg) / 100.0f);
    Serial.print(F("Low voltage reconnect:          ")); Serial.println(serverA.getResponseBuffer(0x900A - reg) / 100.0f);
    Serial.print(F("Under voltage recover:          ")); Serial.println(serverA.getResponseBuffer(0x900B - reg) / 100.0f);
    Serial.print(F("Under voltage warning:          ")); Serial.println(serverA.getResponseBuffer(0x900C - reg) / 100.0f);
    Serial.print(F("Low voltage disconnect:         ")); Serial.println(serverA.getResponseBuffer(0x900D - reg) / 100.0f);
    Serial.print(F("Discharging limit voltage recon:")); Serial.println(serverA.getResponseBuffer(0x900E - reg) / 100.0f);
  }
  else
  {
    Serial.print(F(" ServerA no success register ")); Serial.print(reg, HEX); Serial.print(F(" result=")); Serial.println(result, HEX);
  }

  reg = 0x9013;
  result = serverA.readHoldingRegisters(reg, 15);
  if (result == serverA.ku8MBSuccess) // do something if read is successfull
  {
    Serial.print(F("RTC:                            ")); Serial.println(serverA.getResponseBuffer(0x9013 - reg));
    Serial.print(F("RTC:                            ")); Serial.println(serverA.getResponseBuffer(0x9014 - reg));
    Serial.print(F("RTC:                            ")); Serial.println(serverA.getResponseBuffer(0x9015 - reg));
    Serial.print(F("Equalization charging cycle:    ")); Serial.println(serverA.getResponseBuffer(0x9016 - reg));
    Serial.print(F("Batterie Warning upper Limit:   ")); Serial.println(serverA.getResponseBuffer(0x9017 - reg));
    Serial.print(F("Batterie Warning upper Limit:   ")); Serial.println(serverA.getResponseBuffer(0x9018 - reg));
    Serial.print(F("Controller inner temperature upper limit: ")); Serial.println(serverA.getResponseBuffer(0x9019 - reg)/ 100.0f);
    Serial.print(F("Controller inner temperature upper limit recover: ")); Serial.println(serverA.getResponseBuffer(0x901A - reg)/ 100.0f);
    Serial.print(F("Power component temperature upper limit: ")); Serial.println(serverA.getResponseBuffer(0x901B - reg)/ 100.0f);    
    Serial.print(F("Power component temperature upper limit recover: ")); Serial.println(serverA.getResponseBuffer(0x901C - reg)/ 100.0f);    
    Serial.print(F("Line impedance:                 ")); Serial.println(serverA.getResponseBuffer(0x901D - reg)/ 100.0f);    
    Serial.print(F("Night Time Threshold Volt:      ")); Serial.println(serverA.getResponseBuffer(0x901C - reg)/ 100.0f);    
    Serial.print(F("Light signal startup night delay:")); Serial.println(serverA.getResponseBuffer(0x901F - reg));    
    Serial.print(F("Day Time Threshold Volt:        ")); Serial.println(serverA.getResponseBuffer(0x9020 - reg)/ 100.0f);    
    Serial.print(F("Light signal startup day delay: ")); Serial.println(serverA.getResponseBuffer(0x9021 - reg));    
  }
  else
  {
    Serial.print(F(" ServerA no success register ")); Serial.print(reg, HEX); Serial.print(F(" result=")); Serial.println(result, HEX);
  }

  reg = 0x903D;
  result = serverA.readHoldingRegisters(reg, 3);
  if (result == serverA.ku8MBSuccess) // do something if read is successfull
  {
    Serial.print(F("Load controlling modes:         ")); Serial.println(serverA.getResponseBuffer(0x903D - reg));
    Serial.print(F("Working time length 1:          ")); Serial.println(serverA.getResponseBuffer(0x903E - reg));
    Serial.print(F("Working time length 2:          ")); Serial.println(serverA.getResponseBuffer(0x903F - reg));  
  }
  else
  {
    Serial.print(F(" ServerA no success register ")); Serial.print(reg, HEX); Serial.print(F(" result=")); Serial.println(result, HEX);
  }

  reg = 0x9042;
  result = serverA.readHoldingRegisters(reg, 12);
  if (result == serverA.ku8MBSuccess) // do something if read is successfull
  {
    Serial.print(F("Turn on timing 1 (sec):         ")); Serial.println(serverA.getResponseBuffer(0x9042 - reg));
    Serial.print(F("Turn on timing 1 (min):         ")); Serial.println(serverA.getResponseBuffer(0x9043 - reg));
    Serial.print(F("Turn on timing 1 (hou):         ")); Serial.println(serverA.getResponseBuffer(0x9044 - reg)); 
    Serial.print(F("Turn off timing 1 (sec):        ")); Serial.println(serverA.getResponseBuffer(0x9045 - reg));
    Serial.print(F("Turn off timing 1 (min):        ")); Serial.println(serverA.getResponseBuffer(0x9046 - reg));
    Serial.print(F("Turn off timing 1 (hou):        ")); Serial.println(serverA.getResponseBuffer(0x9047 - reg)); 
    Serial.print(F("Turn on timing 2 (sec):         ")); Serial.println(serverA.getResponseBuffer(0x9048 - reg));
    Serial.print(F("Turn on timing 2 (min):         ")); Serial.println(serverA.getResponseBuffer(0x9049 - reg));
    Serial.print(F("Turn on timing 2 (hou):         ")); Serial.println(serverA.getResponseBuffer(0x904A - reg)); 
    Serial.print(F("Turn off timing 2 (sec):        ")); Serial.println(serverA.getResponseBuffer(0x904B - reg));
    Serial.print(F("Turn off timing 2 (min):        ")); Serial.println(serverA.getResponseBuffer(0x904C - reg));
    Serial.print(F("Turn off timing 2 (hou):        ")); Serial.println(serverA.getResponseBuffer(0x904D - reg));
  }
  else
  {
    Serial.print(F(" ServerA no success register ")); Serial.print(reg, HEX); Serial.print(F(" result=")); Serial.println(result, HEX);
  }

  reg = 0x9065;
  result = serverA.readHoldingRegisters(reg, 1);
  if (result == serverA.ku8MBSuccess)
  {
    Serial.print(F("Length of Night:                ")); Serial.println(serverA.getResponseBuffer(0x9065 - reg));
  }
  else
  {
    Serial.print(F(" ServerA no success register ")); Serial.print(reg, HEX); Serial.print(F(" result=")); Serial.println(result, HEX);
  }

  reg = 0x9067;
  result = serverA.readHoldingRegisters(reg, 1);
  if (result == serverA.ku8MBSuccess)
  {
    Serial.print(F("Battery rated voltage code:     ")); Serial.println(serverA.getResponseBuffer(0x9067 - reg));
  }
  else
  {
    Serial.print(F(" ServerA no success register ")); Serial.print(reg, HEX); Serial.print(F(" result=")); Serial.println(result, HEX);
  }

  reg = 0x9069;
  result = serverA.readHoldingRegisters(reg, 7);
  if (result == serverA.ku8MBSuccess)
  {
    Serial.print(F("Load timing control selection:  ")); Serial.println(serverA.getResponseBuffer(0x9069 - reg));
    Serial.print(F("Default load on/off:            ")); Serial.println(serverA.getResponseBuffer(0x906A - reg));
    Serial.print(F("Equalize Duration:              ")); Serial.println(serverA.getResponseBuffer(0x906B - reg));
    Serial.print(F("Boost Duration:                 ")); Serial.println(serverA.getResponseBuffer(0x906C - reg));
    Serial.print(F("Discharge percentage:           ")); Serial.println(serverA.getResponseBuffer(0x906D - reg)/100.0f);
    Serial.print(F("Charging percentage:            ")); Serial.println(serverA.getResponseBuffer(0x906E - reg)/100.0f);
    Serial.print(F("Management Mode of battery:     ")); Serial.println(serverA.getResponseBuffer(0x9070 - reg));
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

  reg = 0x2000;
  result = serverA.readDiscreteInputs(reg, 1);
  if (result == serverA.ku8MBSuccess)
  {
    Serial.print(F("Over temperature inside the device: ")); Serial.println(serverA.getResponseBuffer(0x2000 - reg));
  }
  else
  {
    Serial.print(F(" ServerA no success register ")); Serial.print(reg, HEX); Serial.print(F(" result=")); Serial.println(result, HEX);
  }

  reg = 0x200C;
  result = serverA.readDiscreteInputs(reg, 1);
  if (result == serverA.ku8MBSuccess)
  {
    Serial.print(F("Day/Night:                      ")); Serial.println(serverA.getResponseBuffer(0x200C - reg));
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
  requestParameter();
}

void loop()
{
  requestData();
}