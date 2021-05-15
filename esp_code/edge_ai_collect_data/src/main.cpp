#include "config.h"
#include "helpers.h"

// Function Headers
void callbackMQTT(char* topic, byte* payload, unsigned int length);
void send_status(void);
void sampleData(unsigned int duration, unsigned int freq, const char* label);
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

  // Wifi Object
  WiFi.mode(WIFI_STA);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname(hostname);
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
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
  Serial.println("BMP Sensor found!");


  // Send status message on client connect
  send_status();
       

  // Turn off setup led
  leds[0] = CRGB::Black;
  FastLED.show();

}

void loop() {
  client.loop();
  vTaskDelay(100/portTICK_PERIOD_MS);

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


//*****************************************************************************

void send_status(void){
  /*
    Sends status message to MQTT
  */
    // Status led to Red while sending status
    leds[0] = CRGB::Red;
    FastLED.show();

    StaticJsonDocument<200> doc;
    double bat_vol = 0.0;
    float temp, pres;
    String output;

    // Get battery voltage
    bat_vol = floor((get_batt_vol() * 100) + 0.5) / 100;

    // Get data from bmp
    temp = round(bmp.readTemperature());
    pres = round(bmp.readPressure());

    doc["user"] = user_name;
    doc["device"] = device_name;
    doc["batt_vol"] = bat_vol;
    doc["temp"] = temp;
    doc["pressure(pa)"] = pres;

    serializeJson(doc, output);
    
    // Convert string to char
    char char_array[output.length() + 1];
    strcpy(char_array, output.c_str());

    // Publish to mqtt
    client.publish(topic_status, char_array);

    // Turn off status led
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


  double bat_vol = 0.0;
  float temp, pres;

  // Get battery voltage
  bat_vol = floor((get_batt_vol() * 100) + 0.5) / 100;

  // Get data from bmp
  temp = round(bmp.readTemperature());
  pres = round(bmp.readPressure());

  // Status data
  doc["user"] = user_name;
  doc["device"] = device_name;
  doc["batt_vol"] = bat_vol;
  doc["temp"] = temp;
  doc["pressure(pa)"] = pres;

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
  
  // Convert string to char
  char char_array[output.length() + 1];
  strcpy(char_array, output.c_str());

  // Print out free stack before send 
  UBaseType_t uxHighWaterMark;
  uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
  Serial.println(uxHighWaterMark);

  // Publish message to mqtt
  client.publish(topic_data, char_array);

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
  // Initialize MPU
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1);
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  mpu.setCycleRate(MPU6050_CYCLE_40_HZ);
  Serial.println("");
}