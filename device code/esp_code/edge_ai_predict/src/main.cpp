#include "config.h"


void setup() {
#ifdef DEBUG
  Serial.begin(115200);
#endif
  // Start serial for debug purpose

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
    delay(250);
#ifdef DEBUG
    Serial.println("Connecting to WiFi..");
#endif
  }
#ifdef DEBUG
  Serial.println("Connected to the WiFi network");
#endif
  
  // Connect to MQTT 
  client.setServer(mqttServer, mqttPort);

  while (!client.connected()) {
#ifdef DEBUG
      Serial.println("Connecting to MQTT...");
#endif
      // Connect to client
      if (client.connect("ESP32TestClient")) {
#ifdef DEBUG  
        Serial.println("Connected");
#endif
      } else {
#ifdef DEBUG
        Serial.print("Failed with state ");
        Serial.print(client.state());
#endif
        delay(1000);
      }
  }

  // Setup MPU
  initializeMPU(mpu);

  // Setup BMP TODO
  if(!bmp.begin())
  {
#ifdef DEBUG
    Serial.println("No BMP180 found!");
#endif
  }

#ifdef DEBUG
  Serial.println("BMP Sensor found!");
#endif

  // Turn off setup led
  leds[0] = CRGB::Black;
  FastLED.show();

}

void loop() {
  client.loop();
  // use here the timer here
  send_status();
 
}


void send_status(void){
  /*
    Sends status message to MQTT
  */
  // Status led to plum while sending status TODO
  leds[0] = CRGB::Red;
  FastLED.show();
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
  // Make JSON object

   StaticJsonDocument<500> doc;

  // Get data from bmp 
  float temp = bmp.readTemperature();
  float pres = bmp.readPressure();

  doc["user"] = user_name;
  doc["device"] = device_name;
  doc["model_name_vers"] = model_name;
  doc["temp"] = round(temp);
  doc["pressure(pa)"] = round(pres);
  doc["high_speed"] = result.classification[0].value;
  doc["low_speed"] = result.classification[1].value;
  doc["mid_speed"] = result.classification[2].value;
  doc["off"] = result.classification[3].value;
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
#ifdef DEBUG
       Serial.write(print_buf);
#endif
   }
}

void initializeMPU(Adafruit_MPU6050 mpu_obj){
  // Initialize MPU
  if (!mpu.begin()) {
#ifdef DEBUG
      Serial.println("Failed to find MPU6050 chip");
#endif
    while (1);
  }


#ifdef DEBUG
  Serial.println("MPU6050 Found!");
#endif
  
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  mpu.setCycleRate(MPU6050_CYCLE_40_HZ);
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