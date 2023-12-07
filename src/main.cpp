/*

MAX                         ESP
DI (Driver Input)           19
DE (Driver Enable Input)    4
RE (Receiver Enable Input)  4
RO (Receiver Output)        18


*/
#include <Arduino.h>

#define RS485_PIN_DIR 4
#define RXD2 18
#define TXD2 19

HardwareSerial rs485(1);

#define RS485_WRITE     1
#define RS485_READ      0

 
void setup() {
  Serial.begin(38400);
  rs485.begin(4800, SERIAL_8N1, RXD2, TXD2);
  pinMode(RS485_PIN_DIR,OUTPUT);
  digitalWrite(RS485_PIN_DIR,RS485_READ);

}


void loop() {
 
  // Import vom digitalen Soyo-Zähler-Unit (AC Power Meter) Rx13
  if (rs485.available() >= 8) {
    
     // Warten, bis genügend Daten empfangen wurden (8 Bytes)
     byte data[8];
     for (int i = 0; i < 8; i++) {
      data[i] = rs485.read();  // Daten einlesen
     }
     Serial.print("0x");Serial.print(data[0], HEX);Serial.print(" 0x");Serial.print(data[1], HEX);Serial.print(" ");
     Serial.print("0x");Serial.print(data[2], HEX);Serial.print(" 0x");Serial.print(data[3], HEX);Serial.print(" ");
     Serial.print("0x");Serial.print(data[4], HEX);Serial.print(" 0x");Serial.print(data[5], HEX);Serial.print(" ");
     Serial.print("0x");Serial.print(data[6], HEX);Serial.print(" 0x");Serial.print(data[7], HEX);Serial.println(" ");Serial.println(" ");
     // Überprüfen, ob die ersten 4 Bytes gleich sind
     
    if (data[0] == 0x24 && data[1] == 0x56 && data[2] == 0x00 && data[3] == 0x21) {
 
      byte chk = (byte)(264 - data[1] - data[0]);
      // Leistung in Watt extrahieren

      Serial.print("Bit 4: "); Serial.println(data[4]);
      Serial.print("Bit 5: "); Serial.println(data[5]);

      uint16_t Power = ((data[4] << 8) | data[5]) * 2;  
  
  // Serieller Monitor
      Serial.print("Power: "); Serial.print(Power);
      Serial.print(" CHKSUMME: "); Serial.println(chk);
      Serial.println("");


  
}
}
}