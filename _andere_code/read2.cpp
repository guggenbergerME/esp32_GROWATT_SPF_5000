#include <Arduino.h>
// github link: https://github.com/4-20ma/ModbusMaster
#include <ModbusMaster.h>

/* Modbus stuff */
#define MODBUS_DIR_PIN  4 // connect DR, RE pin of MAX485 to gpio 4
#define MODBUS_RX_PIN 16 // Rx pin - RO 
#define MODBUS_TX_PIN 17 // Tx pin - DI
#define MODBUS_SERIAL_BAUD 9600 // Baud rate for esp32 and max485 communication

// voltage, current and frequency data register of DDM18SD
uint16_t data_register[3] = {0x0000, 0x1000, 0x2000};

//Initialize the ModbusMaster object as node
ModbusMaster node;

// Pin 4 made high for Modbus transmision mode
void modbusPreTransmission()
{
  delay(500);
  digitalWrite(MODBUS_DIR_PIN, HIGH);
}
// Pin 4 made low for Modbus receive mode
void modbusPostTransmission()
{
  digitalWrite(MODBUS_DIR_PIN, LOW);
  delay(500);
}


void setup()
{
  //  esp serial communication
  Serial.begin(115200);
  pinMode(MODBUS_DIR_PIN, OUTPUT);
  digitalWrite(MODBUS_DIR_PIN, LOW);

  //Serial2.begin(baud-rate, protocol, RX pin, TX pin);.
  Serial2.begin(MODBUS_SERIAL_BAUD, SERIAL_8E1, MODBUS_RX_PIN, MODBUS_TX_PIN);
  Serial2.setTimeout(200);
  //modbus slave ID 14
  node.begin(1, Serial2);

//  callbacks allow us to configure the RS485 transceiver correctly
   node.preTransmission(modbusPreTransmission);
   node.postTransmission(modbusPostTransmission);
  
  }

void loop()
{
    uint8_t result;
    uint16_t data[2];
    int i;
    float reading;

for(i=0; i<=2; i++){
Serial.print(node.readInputRegisters(data_register[i], 1));
delay(200);
Serial.println(" ...");
}
/*
    for(i=0; i<=2; i++){
      //Modbus function 0x04 Read Input Registers according to energy meter datasheet
      result = node.readInputRegisters(data_register[i], 1);
        if (result == node.ku8MBSuccess) {
          Serial.println("Success, Received data: ");
          
          //Retrieve the data from getResponseBuffer(uint8_t u8Index) function.
          //that is return 16-bit data. our energy meter return 32-bit data everytime.
          //that's why, we take the value to a array called data
           data[0]=node.getResponseBuffer(0x00);
           data[1]=node.getResponseBuffer(0x01);
           
           //read voltage
           if(data_register[i] == 0x0000){
            Serial.print("Volatge: ");
            // we just convert the uint16_t type data array to float type using type casting
            reading = *((float *)data);
            Serial.print(reading);
            Serial.println(" Volt");
           }
           //read current
           if(data_register[i] == 0x0008){
            Serial.print("Current: ");
             // we just convert the uint16_t type data array to float type using type casting
            reading = *((float *)data);
            Serial.print(reading);
            Serial.println(" Amps");
           }
           //read Frequency
           if(data_register[i] == 0x002A){
            Serial.print("Frequency: ");
             // we just convert the uint16_t type data array to float type using type casting
            reading = *((float *)data);
            Serial.print(reading);
            Serial.println(" Hz");
           }
        } else {
          Serial.print("Failed, Response Code: ");
          Serial.print(result, HEX);
          Serial.println("");
          delay(5000); 
        }
      
    }
*/
    delay(1000);
  }