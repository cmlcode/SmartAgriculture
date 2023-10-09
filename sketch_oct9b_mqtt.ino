#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include "DFRobot_SHT40.h"

DFRobot_SHT40 SHT40(SHT40_AD1B_IIC_ADDR);

// Set your WiFi credentials
const char* ssid = "liberty";
const char* password = "12345678";
const char* mqttServer = "172.20.10.3";
const int mqttPort = 1883;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

uint32_t id = 0;
float temperature;
int humidityValue;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
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
  reconnect();
}

void loop() {
  mqttClient.loop();

  // Get humidity value from A0 pin
  humidityValue = analogRead(A0);

  // Get temperature from SHT40
  temperature = SHT40.getTemperature(PRECISION_HIGH);
  if(temperature == MODE_ERR) {
    Serial.println("Incorrect mode configuration to get temperature");
  } else {
    Serial.print("Temperature :"); Serial.print(temperature); Serial.println(" C");
  }

  // Publish temperature and humidity values to MQTT broker
  String tempPayload = String(temperature);
  // mqttClient.publish("esp32/environment", tempPayload.c_str());
  String humidityPayload = String(humidityValue);
  String envValues = tempPayload +" " + humidityPayload;
  mqttClient.publish("esp32/environment", envValues.c_str());

  delay(3000); // Publish every 3 seconds
}

void reconnect() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (mqttClient.connect("ESP32Client")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}
