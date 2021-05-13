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

// Timer setup
//static TimerHandle_t status_timer = NULL;

// Device Names
const char* user_name = "cristianSulighetean";
const char* device_name = "esp32_train_test";
const char* topic_status = "cristianSulighetean/esp32_train_test/status";
const char* topic_data = "cristianSulighetean/esp32_train_test/data";

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

// MQTT cloud server
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


// Function Headers
void callbackMQTT(char* topic, byte* payload, unsigned int length);
void send_status(void);
void sampleData(unsigned int duration, unsigned int freq, const char* label);
void statusTimerCallback(TimerHandle_t xTimer);
void initializeMPU(Adafruit_MPU6050 mpu_obj);
double get_batt_vol(void);


void setup(void) {

  // Start serial for debug purpose
  Serial.begin(115200);

  // Create mutex
  mutex = xSemaphoreCreateMutex();

  // Setup status LED
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness(150);

  // Set Status led to blue
  leds[0] = CRGB::Yellow;
  FastLED.show();

  // // Setup timers
  // status_timer = xTimerCreate(
  //                     "Status timer",               // Name of timer
  //                     5000 / portTICK_PERIOD_MS,    // Period of timer (in ticks)
  //                     pdTRUE,                       // Auto-reload
  //                     (void *)0,                    // Timer ID
  //                     statusTimerCallback);         // Callback function


  // // Check to make sure timers were created
  // if (status_timer == NULL) 
  //   Serial.println("Could not create the status timer");

  // Wifi Object
  WiFi.mode(WIFI_STA);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname(hostname);
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  
  // Connect to MQTT 
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callbackMQTT);

  while (!client.connected()) {
      Serial.println("Connecting to MQTT...");

      // Connect to client
      if (client.connect("ESP32TestClient")) {
  
        Serial.println("Connected");

        // Start status timer
        //Serial.println("Starting timers...");
        //vTaskDelay(500 / portTICK_PERIOD_MS);

        // Start timers (max block time if command queue is full)
        //xTimerStart(status_timer, portMAX_DELAY);
      
      } else {
        Serial.print("Failed with state ");
        Serial.print(client.state());
        delay(2000);
      }
  }

  //  MQTT Subscribe to topics
  client.subscribe("esp/request");

  
  // Setup MPU
  initializeMPU(mpu);

  // Setup BMP
  if(!bmp.begin())
  {
    Serial.println("No BMP180 found!");
  }

  // Set Status led to blue
  leds[0] = CRGB::Black;
  FastLED.show();

}

void loop() {
  client.loop();
}

//*****************************************************************************
// Callbacks

void callbackMQTT(char* topic, byte* payload, unsigned int length) {
 
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
 
  // Prepare conversion from byte to char
  char payloadString[length+1];
  memcpy(payloadString, payload, length);
  payloadString[length] = '\0';

  // Deserialize message 
  StaticJsonDocument<100> doc;
  DeserializationError error = deserializeJson(doc, payloadString, length+1);

  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  Serial.print("Message:");
  
  // Json content
  //const char* device = doc["device"]; // "esp1"
  const char* label = doc["label"]; // "test_acc"
  int duration = doc["duration"]; // 1
  int freq = doc["freq"]; // 100

  Serial.println("Status message recieved");

  // Sample request data
  sampleData(duration, freq, label);
}


void statusTimerCallback(TimerHandle_t xTimer){
  // Print message if timer 0 expired
  if ((uint32_t)pvTimerGetTimerID(xTimer) == 0) {
    // Call send status 
    send_status();
  }

}

//*****************************************************************************

void send_status(void){
  /*
    Sends status message to MQTT
  */

  // Status led to Red while sending status
  leds[0] = CRGB::Red;
  FastLED.show();

  StaticJsonDocument<200> doc;

  // Get battery voltage
  double bat_vol = floor((get_batt_vol() * 100) + 0.5) / 100;

  // Get data from bmp
  float temp = round(bmp.readTemperature());
  float pres = round(bmp.readPressure());

  doc["user"] = user_name;
  doc["device"] = device_name;
  doc["batt_vol"] = bat_vol;
  doc["temp"] = temp;
  doc["pressure(pa)"] = pres;

  String output;
  serializeJson(doc, output);
  
  // Convert string to char
  char char_array[output.length() + 1];
  strcpy(char_array, output.c_str());

  // Topic will be username/device/status
  client.publish(topic_status, char_array);

  // Reset status
  leds[0] = CRGB::Black;
  FastLED.show();

}


void sampleData(unsigned int duration, unsigned int freq, const char* label){
  /*
  Input 
    freq
    duration (in seconds)
  This function samples the data and encodes it in a JSON
  As it samples the data it also adds it to the json
  */


  // Take mutex
  xSemaphoreTake(mutex, portMAX_DELAY);
  Serial.println("Sending data, taking mutex!");

  // Status led to green while sending sample
  leds[0] = CRGB::Green;
  FastLED.show();

  // Setup freq and duration parameters
  unsigned int interval_ms = (1000 / (freq + 1));

  DynamicJsonDocument doc(ESP.getMaxAllocHeap());

  // Insert metadata
  doc["label"] = label;
  doc["duration"] = duration*1000;
  doc["freq"] = freq;

  JsonArray values = doc.createNestedArray("values");

  // Get current time 
  unsigned int current_time = millis();

  // Sample loop
  while (millis() <= current_time + (duration * 1000)){
    static unsigned long last_interval_ms = 0;
    sensors_event_t a, g, temp;

    // Send data for Data Forwarder
    if (millis() > last_interval_ms + interval_ms) {
      last_interval_ms = millis();
      mpu.getEvent(&a, &g, &temp);

      // Add values to the json object
      JsonArray values_0 = values.createNestedArray();
      values_0.add(a.acceleration.x);
      values_0.add(a.acceleration.y);
      values_0.add(a.acceleration.z);
    }
  }

  String output;
  serializeJson(doc, output);
  //Serial.println(output);
  
  // Convert string to char
  char char_array[output.length() + 1];
  strcpy(char_array, output.c_str());

  // Print out free stack before send 
  UBaseType_t uxHighWaterMark;
  uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
  Serial.println(uxHighWaterMark);

  // Call the send data function
  Serial.println(client.publish(topic_data, char_array));
  Serial.println("Data was send");

  // Print out free stack after send 
  uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
  Serial.println(uxHighWaterMark);

  // Reset status
  leds[0] = CRGB::Black;
  FastLED.show();

  Serial.println("Data send, giving mutex!");
  xSemaphoreGive(mutex);
}


double get_batt_vol(void){
  /*
  Read battery level and return an int
  Set up the sensor also when calling
  */
  double bat_vol = floor((BL.getBatteryVolts() * 100) + 0.5) / 100;
  //Serial.println(bat_vol);

  return bat_vol;
}

void initializeMPU(Adafruit_MPU6050 mpu_obj){
  // Setup of the MPU
  // Initialize MPU
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1);
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  mpu.setCycleRate(MPU6050_CYCLE_40_HZ);
  Serial.print("Cycle rate set to: 40Hz");

  Serial.println("");
  delay(10);
}