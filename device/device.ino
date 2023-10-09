#include "DFRobot_SHT40.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>

DFRobot_SHT40 SHT40(SHT40_AD1B_IIC_ADDR);

uint32_t sensor_id = 0;
uint32_t device_id = 0;
float temperature;
int humidityValue;

const char *ssid = "esp32";
const char *password = "123456789";

WiFiServer server(80);

uint32_t get_device_id(uint32_t *device_id){
  return 2;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(57600);
  if (device_id == 0){
    device_id = get_device_id(&device_id);
  }
  // Initialize the SHT40 sensor
  SHT40.begin();
  while((sensor_id = SHT40.getDeviceID()) == 0){
    Serial.println("ID retrieval error, please check whether the device is connected correctly!!!");
    delay(1000);
  }
  Serial.print("id :0x"); Serial.println(sensor_id, HEX);

  // Configure WiFi
  Serial.println("Configuring access point...");
  WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  server.begin();
  Serial.println("Server started");
}

void loop() {
  // Get humidity value from A0 pin
  humidityValue = analogRead(A0);

  // Get temperature from SHT40
  temperature = SHT40.getTemperature(PRECISION_HIGH);
  if(temperature == MODE_ERR){
    Serial.println("Incorrect mode configuration to get temperature");
  } else{
    Serial.print("Temperature :"); Serial.print(temperature); Serial.println(" C");
  }

  // Activate heater if humidity value is above a certain threshold
  if(humidityValue > 700){
    SHT40.enHeater(POWER_CONSUMPTION_H_HEATER_1S);
  }

  WiFiClient client = server.available();
  if (client) {
    Serial.println("New Client.");
    String currentLine = "";
    while (client.connected()) {
      char c = client.read();
      Serial.write(c);
      if (c == '\n') {
        if (currentLine.length() == 0) {
          client.println("HTTP/1.1 200 OK");
          client.println("Content-type:text/html");
          client.println();
          client.print(device_id);
          client.print(" ");
          client.print(humidityValue);
          client.print(" ");
          client.println(temperature);
          // client.print("Click <a href=\"/H\">here</a> to turn ON the LED.<br>");
          // client.print("Click <a href=\"/L\">here</a> to turn OFF the LED.<br>");
          // if (currentLine.endsWith("GET /H")) {
          //   client.print("Humidity Value from A0: "); client.println(humidityValue);
          //   client.print("Temperature :"); client.print(temperature); client.println(" C");
          // }
          client.println();
          break;
        } else {
          currentLine = "";
        }
      } else if (c != '\r') {
        currentLine += c;
      }
      if (currentLine.endsWith("GET /L")) {
        digitalWrite(LED_BUILTIN, LOW);
      }
    }
    client.stop();
    Serial.println("Client Disconnected.");
  }
  delay(1000);
  Serial.println("----------------------------------------");
}
