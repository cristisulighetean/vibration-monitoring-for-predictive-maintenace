#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>

#include <FastLED.h>
#include <Adafruit_BMP085.h>

#include <Pangodream_18650_CL.h>
#include <cmath>

// JSON Library
#define ARDUINOJSON_USE_DOUBLE 1
#include <ArduinoJson.h>

// Core definitions 
static const BaseType_t pro_cpu = 0;
static const BaseType_t app_cpu = 1;

// Mutex for send data 
static SemaphoreHandle_t mutex;

// Device Names
const char* user_name = "cristianSulighetean";
const char* device_name = "esp32_train_test";
char* topic_status = "cristianSulighetean/esp32_train_test/status";
char* topic_data = "cristianSulighetean/esp32_train_test/data";

// Wifi Credentials
const char* ssid = "HustleHub";
const char* password = "clujnapoca31";

// Wifi Hostname 
const char* hostname = "Predictedge Device";

WiFiClient espClient;
PubSubClient client(espClient);

// MQTT Server TODO
IPAddress mqttServer(192, 168, 1, 5);
const int mqttPort = 1883;

// MQTT cloud server TODO
//const char* mqttUser = "yourMQTTuser";
//const char* mqttPassword = "yourMQTTpassword";

// MPU object
Adafruit_MPU6050 mpu;
// BMP180 sensor
Adafruit_BMP085 bmp;

// FastLED setup
#define NUM_LEDS 1
#define DATA_PIN 32
CRGB leds[NUM_LEDS];

// Batter Measurment
#define ADC_PIN 36
#define CONV_FACTOR 1.71
#define READS 50
Pangodream_18650_CL BL(ADC_PIN, CONV_FACTOR, READS);

