#include <SoftwareSerial.h>
#include <ModbusMaster.h>

#define RX_PIN 17 // connect to converter's RX wire
#define TX_PIN 16 // connect to converter's TX wire
#define MODBUS_DEVICE_ID 1
SoftwareSerial swSerial(RX_PIN, TX_PIN);
ModbusMaster sensor;

const char* ssid = "home";
const char* password = "???";
const char* mqttServer = "192.168.1.5";
const int mqttPort = 1883;
const char* mqttUser = "mqtt";
const char* mqttPassword = "???";

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  Serial.begin(9600);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(5000);
    Serial.println("Connecting to WiFi..");
  }
  
  Serial.println("Connected to the WiFi network");
  
  swSerial.begin(9600);
  sensor.begin(MODBUS_DEVICE_ID, swSerial);
    
  client.setServer(mqttServer, mqttPort);
  
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");    
    if (client.connect("ESP32Client", mqttUser, mqttPassword )) {    
      Serial.println("connected");    
    } else {    
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(10000);
    }
  }
}

void loop() {
  client.loop();
  readFlow();
  delay(3000);
  if (!client.connected()) {
    delay(60000);
    if (!client.connected()) {
      ESP.restart();
    }    
  }
}

void readFlow() {
  uint8_t result;
  uint16_t buf[2];
  float flow;
  result = sensor.readHoldingRegisters(1, 2);
  if (result == sensor.ku8MBSuccess)
  {
    buf[1] = sensor.getResponseBuffer(0);
    buf[0] = sensor.getResponseBuffer(1);
    memcpy(&flow, &buf, sizeof(float));
    client.publish("flowmeter/flow",  String(flow).c_str());
  }
}