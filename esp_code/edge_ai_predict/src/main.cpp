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


void setup() {
  // Start serial for debug purpose
  Serial.begin(115200);

  //Setup status LED 
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness(150);

  //Set Status led to blue
  leds[0] = CRGB::Yellow;
  FastLED.show();

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

  while (!client.connected()) {
      Serial.println("Connecting to MQTT...");

      // Connect to client
      if (client.connect("ESP32TestClient")) {
  
        Serial.println("Connected");

      } else {
        Serial.print("Failed with state ");
        Serial.print(client.state());
        delay(1000);
      }
  }

  // Setup MPU
  initializeMPU(mpu);

  // Setup BMP TODO
  if(!bmp.begin())
  {
     Serial.println("No BMP180 found!");
  }
  Serial.println("BMP Sensor found!");

  // Turn off setup led
  leds[0] = CRGB::Black;
  FastLED.show();

}

void loop() {
  client.loop();
  // use here the timer here
  send_status();
  vTaskDelay(5000 / portTICK_PERIOD_MS);
}


void send_status(void){
  /*
    Sends status message to MQTT
  */
  // Status led to plum while sending status TODO
  leds[0] = CRGB::Red;
  FastLED.show();

  StaticJsonDocument<250> doc;

  // Get data from bmp TODO
  float temp = bmp.readTemperature();
  float pres = bmp.readPressure();

  doc["user"] = user_name;
  doc["device"] = device_name;
  doc["temp"] = round(temp);
  doc["pressure(pa)"] = round(pres);
  

  // Call predict function
  ei_impulse_result_t result = get_prediction();

  // Switch color to Red inside send message function 
  leds[0] = CRGB::Red;
  FastLED.show();
  //********************************************************
  // print the predictions
    ei_printf("Predictions ");
    ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
        result.timing.dsp, result.timing.classification, result.timing.anomaly);
    ei_printf(": \n");
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
    }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("    anomaly score: %.3f\n", result.anomaly);
#endif

//***************************************************************
  
  // Get prediction result
  doc["normal"] = result.classification[0].value;
  doc["off"] = result.classification[1].value;
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    doc["anomaly_score"] = result.anomaly;
#else
    doc["anomaly_score"] = -1;
#endif
  
  String output;
  serializeJson(doc, output);
  
  // Convert string to char
  char char_array[output.length() + 1];
  strcpy(char_array, output.c_str());

  // Topic will be username/device/status
  client.publish(topic_predict, char_array);

  // Reset status 
  leds[0] = CRGB::Black;
  FastLED.show();
}


ei_impulse_result_t get_prediction(void)
{
  // Set status LED to green while doing a prediction
  leds[0] = CRGB::Green;
  FastLED.show();

  // Do Prediction
    ei_printf("Sampling...\n");

    // Allocate a buffer here for the values we'll read from the IMU
    float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };
    sensors_event_t a, g, temp;


    for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += 3) {
        // Determine the next tick (and then sleep later)
        uint64_t next_tick = micros() + (EI_CLASSIFIER_INTERVAL_MS * 1000);

        mpu.getEvent(&a, &g, &temp);

        buffer[ix + 0] = a.acceleration.x;
        buffer[ix + 1] = a.acceleration.x;
        buffer[ix + 2] = a.acceleration.x;

        delayMicroseconds(next_tick - micros());
    }

    // Turn the raw buffer in a signal which we can the classify
    signal_t signal;
    int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0) {
        ei_printf("Failed to create signal from buffer (%d)\n", err);
    }

    // Run the classifier
    ei_impulse_result_t result = { 0 };

    // Print any errors
    err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
    }

    // Reset status 
    leds[0] = CRGB::Black;
    FastLED.show();


    // Return the ei_impulse_result_t object
    return result;
}

void ei_printf(const char *format, ...) {
   static char print_buf[1024] = { 0 };

   va_list args;
   va_start(args, format);
   int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
   va_end(args);

   if (r > 0) {
       Serial.write(print_buf);
   }
}

void initializeMPU(Adafruit_MPU6050 mpu_obj){
  // Initialize MPU
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1);
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  mpu.setCycleRate(MPU6050_CYCLE_40_HZ);
  delay(10);
}


float round(float var)
{
    // 37.66666 * 100 =3766.66
    // 3766.66 + .5 =3767.16    for rounding off value
    // then type cast to int so value is 3767
    // then divided by 100 so the value converted into 37.67
    float value = (int)(var * 100 + .5);
    return (float)value / 100;
}