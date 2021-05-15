#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <cmath>

#include <FastLED.h>
#include <Adafruit_BMP085.h>

#include <fan-test-project_last_inference.h>

// JSON Library
#define ARDUINOJSON_USE_DOUBLE 1
#include <ArduinoJson.h>

// Core definitions 
static const BaseType_t pro_cpu = 0;
static const BaseType_t app_cpu = 1;

// Device Registration
const char* user_name = "cristianSulighetean";
const char* device_name = "esp32_predict_test";
const char* topic_predict = "cristianSulighetean/esp32_train_test/predict_result";

// Define credentials for WIFI & MQTT
const char* ssid = "HustleHub";
const char* password = "clujnapoca31";

// Wifi Hostname 
const char* hostname = "Predictedge Device";
WiFiClient espClient;
PubSubClient client(espClient);

// MQTT Server TODO
IPAddress mqttServer(192, 168, 1, 5);
const int mqttPort = 1883;

// MQTT cloud server
//const char* mqttUser = "yourMQTTuser";
//const char* mqttPassword = "yourMQTTpassword";

// MPU object
Adafruit_MPU6050 mpu;
// BMP180 sensor
Adafruit_BMP085 bmp;

// FastLED status TODO choose pins
#define NUM_LEDS 1
#define DATA_PIN 32
CRGB leds[NUM_LEDS];

// Function headers
void send_status(void);
void predictTimerCallback(TimerHandle_t xTimer);
void initializeMPU(Adafruit_MPU6050 mpu_obj);
void ei_printf(const char *format, ...);
ei_impulse_result_t get_prediction(void);


/* Private variables ------------------------------------------------------- */
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
