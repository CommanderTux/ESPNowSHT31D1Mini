/*
 ESP-NOW based sensor station

 Sends readings to an ESP-Now server with a fixed mac address

 Original author: Anthony Elder
 License: Apache License v2
 
 December 2018 Modified by MarcellS2
 
 Uses Adafruit SHT31 Library
 For Lolin ESP8266 Board or similar ESP8266 modules
 Connect the pins in the following way:
 *  Wemos esp-wroom-02 Pocket 8266 D1
 *  Wemos D1 R1 board type
 ESP-NOW based sensor using a BME280 temperature/pressure/humidity sensor

 * ==================================
 *   GPIO5              SCL (pin D1)
 *   GPIO4              SDA (pin D2)
 *   3.3V               VDD (pin 3V3)
 *   GND                GND (pin GND)
 *
 * ====================================
 * 
 * These pins are the same as D1 & D2 on a Lolin based board 
 * 
*/
#include <Arduino.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include "Adafruit_SHT31.h"
#include <SPI.h>

bool enableHeater = false;

extern "C" {
  #include <espnow.h>
}

// this is the MAC Address of the remote ESP server which receives these sensor readings
uint8_t remoteMac[] = {0x24, 0x6F, 0x28, 0xAB, 0xBC, 0x49};

#define WIFI_CHANNEL 4
#define SLEEP_SECS 15 * 60 // 15 minutes
#define SEND_TIMEOUT 245  // 245 millis seconds timeout 

// keep in sync with slave struct
struct __attribute__((packed)) SENSOR_DATA {
    float temp;
    float humidity;
    float pressure;
    float voltage;
} sensorData;

//Create an instance of the object
Adafruit_SHT31 sht31 = Adafruit_SHT31();

volatile boolean callbackCalled;

const int analogInPin = A0;  // ESP8266 Analog Pin ADC0 = A0
int sensorValue = 0; 

void readSHT31() {
  sensorData.temp = sht31.readTemperature();
  sensorData.humidity = sht31.readHumidity();
  sensorData.voltage = (analogRead(analogInPin)/235.0 );
  Serial.printf("temp=%01f, humidity=%01f, pressure=%01f, voltage=%01f\n", sensorData.temp, sensorData.humidity, sensorData.pressure, sensorData.voltage);
}

void gotoSleep() {
  // add some randomness to avoid collisions with multiple devices
  int sleepSecs = SLEEP_SECS + ((uint8_t)RANDOM_REG32/2); 
  Serial.printf("Up for %i ms, going to sleep for %u secs...\n", millis(), sleepSecs); 
  ESP.deepSleep(sleepSecs * 1000000, RF_NO_CAL);
}

void setup() {
  Serial.begin(115200); Serial.println();

  Wire.begin();
  Wire.setClock(400000); //Increase to fast I2C speed!

  Serial.println("SHT31 test");
  if (! sht31.begin(0x45)) {   // Set to 0x45 for alternate i2c addr
    Serial.println("Couldn't find SHT31");
    while (1) delay(1);
  }

  readSHT31();

  WiFi.mode(WIFI_STA); // Station mode for esp-now sensor node
  WiFi.disconnect();

  Serial.printf("This mac: %s, ", WiFi.macAddress().c_str()); 
  Serial.printf("target mac: %02x%02x%02x%02x%02x%02x", remoteMac[0], remoteMac[1], remoteMac[2], remoteMac[3], remoteMac[4], remoteMac[5]); 
  Serial.printf(", channel: %i\n", WIFI_CHANNEL); 

  if (esp_now_init() != 0) {
    Serial.println("*** ESP_Now init failed");
    gotoSleep();
  }
  Serial.println(" -- ESP_Now init success --");

  
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_add_peer(remoteMac, ESP_NOW_ROLE_SLAVE, WIFI_CHANNEL, NULL, 0);

  esp_now_register_send_cb([](uint8_t* mac, uint8_t sendStatus) {
    Serial.printf("send_cb, send done, status = %i\n", sendStatus);
    callbackCalled = true;
  });

  callbackCalled = false;

  uint8_t bs[sizeof(sensorData)];
  memcpy(bs, &sensorData, sizeof(sensorData));
  esp_now_send(NULL, bs, sizeof(sensorData)); // NULL means send to all peers
}

void loop() {
  if (callbackCalled || (millis() > SEND_TIMEOUT)) {
    gotoSleep();
  }
}

