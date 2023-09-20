#include <Arduino.h>
#include "DFRobot_SHT40.h"

#define humidityPin A0
#define delayVal 1000
#define optimalHumidity 600
#define humidityRange 100

uint32_t humidity;
uint32_t id = 0;
float temperature;

bool needWater(uint32_t humidity);

DFRobot_SHT40 SHT40(SHT40_AD1B_IIC_ADDR);

void setup() {
  Serial.begin(57600);
  SHT40.begin();
  while ((id = SHT40.getDeviceID()) == 0) {
    Serial.println("ID retrieval error, please check whether device is connected correctly");
    delay(delayVal);
  }

  delay(delayVal);
  Serial.print("id :0x");
  Serial.println(id, HEX);
}

void loop() {
  temperature = SHT40.getTemperature(PRECISION_HIGH);
  if (temperature == MODE_ERR) {
    Serial.println("Incorrect mode configuration to get temperature");
  } else {
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" C");
  }
  humidity = analogRead(humidityPin);
  if (needWater(humidity)) {
    Serial.println("Water me");
  }
  delay(delayVal);
  Serial.println("-----------------");
}

bool needWater(uint32_t humidity) {
  if (humidity < (optimalHumidity - humidityRange)) return true;
  return false;
}