#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define FREQUENCY_HZ        130
#define INTERVAL_MS         (1000 / (FREQUENCY_HZ+1))

// can be overwritten when calling edge-impulse-data-forwarder
// edge-impulse-data-forwarder --frequency 100
// set different baudrate
// edge-impulse-data-forwarder --baud-rate 460800

// MPU object
Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); \

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1);
  }
  Serial.println("MPU6050 Found!");


  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
  
  mpu.setCycleRate(MPU6050_CYCLE_40_HZ);
  Serial.print("Cycle rate set to: 40Hz");

  Serial.println("");
  delay(10);
}

void loop() {
  
  static unsigned long last_interval_ms = 0;
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Send data for Data Forwarder
  if (millis() > last_interval_ms + INTERVAL_MS) {
    last_interval_ms = millis();
    mpu.getEvent(&a, &g, &temp);

    Serial.print(a.acceleration.x);
    Serial.print('\t');
    Serial.print(a.acceleration.y);
    Serial.print('\t');
    Serial.println(a.acceleration.z);
  }
}