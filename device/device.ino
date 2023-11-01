#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include "DFRobot_SHT40.h"
#define relaypin 4

DFRobot_SHT40 SHT40(SHT40_AD1B_IIC_ADDR);

// Set your WiFi credentials
const char* ssid = "liberty";
const char* password = "12345678";
const char* mqttServer = "54.206.127.177";
const char* mqttUser = "Jack";
const char* mqttPassword = "123456";
const int mqttPort = 1883;
const int plant_id = 1;
long int open_time = 0;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

uint32_t id = 0;
float temperature;
int humidityValue;
float expext_temperature_low = 0;
float expext_temperature_high = 0;
float expect_humidity_low = 0;
float expect_humidity_high = 0;

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char payloadStr[length + 1];
  memcpy(payloadStr, payload, length);
  payloadStr[length] = '\0';
  Serial.println(payloadStr);
  int expected_plant_id;
  float temp1,temp2,temp3,temp4;
  int ret = sscanf(payloadStr, "%d %f %f %f %f", &expected_plant_id,&temp1,&temp2,&temp3,&temp4);
  if(ret==5){
    if(expected_plant_id==plant_id){
      expect_humidity_low = temp1;
      expect_humidity_high = temp2;
      expext_temperature_low = temp3;
      expext_temperature_high = temp4;
      Serial.println("configuration received: comfort temperature: "+String(expext_temperature_low)+"-"+String(expext_temperature_high)+" humidity: "+String(expect_humidity_low)+"-"+String(expect_humidity_high));
      Serial.println("---------------------------------------------");
    }
  }
}

void reconnect() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (mqttClient.connect("ESP32Client", mqttUser, mqttPassword)) {
      Serial.println("connected");
      if (mqttClient.subscribe("config")) {
        Serial.println("Subscribed to topic: config");
      } else {
        Serial.println("Failed to subscribe");
      }
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  pinMode(relaypin, OUTPUT);
  Serial.begin(115200);
  // Connect to WiFi
  
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("Connected to WiFi!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  // Initialize the SHT40 sensor
  SHT40.begin();
  while((id = SHT40.getDeviceID()) == 0) {
    Serial.println("ID retrieval error, please check whether the device is connected correctly!!!");
    delay(1000);
  }
  Serial.print("id :0x"); Serial.println(id, HEX);
  // Connect to MQTT broker
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(mqttCallback);
  reconnect();
}

void loop() {
  mqttClient.loop();

  // Get humidity value from A0 pin
  humidityValue = analogRead(A0)/100;
  // Get temperature from SHT40
  temperature = SHT40.getTemperature(PRECISION_HIGH);
  if(temperature == MODE_ERR) {
    Serial.println("Incorrect mode configuration to get temperature");
  }
  if (humidityValue < expect_humidity_low){
    digitalWrite(relaypin, 0);
    if (open_time == 0) {open_time = millis();}
  }
  else{
    digitalWrite(relaypin, 255);
    if (open_time != 0) {
      long int session_time = millis() - open_time;
      String irrValues = "1 " + String(session_time);
      mqttClient.publish("esp32/Irrigation", irrValues.c_str());
      open_time = 0;    }
  }
  // Publish temperature and humidity values to MQTT broker
  String tempPayload = String(temperature);
  String humidityPayload = String(humidityValue);
  String envValues = tempPayload +" " + humidityPayload;
  mqttClient.publish("esp32/environment", envValues.c_str());
  delay(3000); // Publish every 3 seconds
  
}
