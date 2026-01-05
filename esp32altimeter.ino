/*
 * ESP32 Combined Sensor Telemetry with LoRa Transmission
 * BNO055 IMU + BMP390 Pressure + GPS + LoRa
 * 
 * PIN CONNECTIONS FOR ESP32-WROOM-32D:
 * 
 * BNO055 IMU (I2C):
 *   SDA -> GPIO21
 *   SCL -> GPIO22
 *   VIN -> 3.3V
 *   GND -> GND
 * 
 * BMP390 Pressure (I2C - shared bus):
 *   SDA -> GPIO21 (same as BNO055)
 *   SCL -> GPIO22 (same as BNO055)
 *   VIN -> 3.3V
 *   GND -> GND
 * 
 * HGLRC M100-PRO GPS (UART):
 *   TX (from GPS) -> GPIO16 (RX2)
 *   RX (to GPS) -> GPIO17 (TX2)
 *   VCC -> 5V
 *   GND -> GND
 * 
 * RYLR998 LoRa (UART):
 *   TX (from LoRa) -> GPIO25 (custom RX)
 *   RX (to LoRa) -> GPIO26 (custom TX)
 *   VCC -> 3.3V
 *   GND -> GND
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>
#include <TinyGPS++.h>
#include <utility/imumaths.h>

// Define I2C pins for ESP32
#define SDA_PIN 21
#define SCL_PIN 22

// Define GPS serial (Hardware Serial 2)
#define GPS_RX 16
#define GPS_TX 17

// Define LoRa serial (Hardware Serial 1)
#define LORA_RX 25
#define LORA_TX 26

// Create sensor objects
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Adafruit_BMP3XX bmp;
TinyGPSPlus gps;

// Serial ports
HardwareSerial gpsSerial(2);  // Use Serial2 for GPS
HardwareSerial loraSerial(1); // Use Serial1 for LoRa

#define SEALEVELPRESSURE_HPA (1013.25)

// LoRa transmission control
unsigned long lastLoraTransmit = 0;
const unsigned long loraTransmitInterval = 100; // Send every 100ms

void testLoRa() {
  Serial.println("\n=== Testing LoRa Module ===");
  
  // Test 1: Basic AT command
  Serial.print("1. Sending AT command... ");
  loraSerial.println("AT");
  delay(200);
  
  if (loraSerial.available()) {
    String response = loraSerial.readStringUntil('\n');
    Serial.print("Response: ");
    Serial.println(response);
    if (response.indexOf("+OK") >= 0) {
      Serial.println("   ✓ LoRa module responding!");
    }
  } else {
    Serial.println("   ✗ No response - check wiring!");
  }
  
  // Configure for transmitter
  Serial.print("2. Setting Network ID to 18... ");
  loraSerial.println("AT+NETWORKID=18");
  delay(200);
  if (loraSerial.available()) {
    Serial.println(loraSerial.readStringUntil('\n'));
  }
  
  Serial.print("3. Setting Address to 0... ");
  loraSerial.println("AT+ADDRESS=0");
  delay(200);
  if (loraSerial.available()) {
    Serial.println(loraSerial.readStringUntil('\n'));
  }
  
  Serial.print("4. Setting Parameters (9,7,1,12)... ");
  loraSerial.println("AT+PARAMETER=9,7,1,12");
  delay(200);
  if (loraSerial.available()) {
    Serial.println(loraSerial.readStringUntil('\n'));
  }
  
  Serial.println("=== LoRa Configuration Complete ===\n");
}

void sendLoraData(String data) {
  // Simple send - data should be small enough now
  int dataLength = data.length();
  
  String command = "AT+SEND=1," + String(dataLength) + "," + data;
  loraSerial.println(command);
  
  // Clear any responses quickly
  delay(30);
  while (loraSerial.available()) {
    loraSerial.read();
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("ESP32 Combined Sensor Telemetry with LoRa");
  Serial.println("==========================================");
  
  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // Initialize BNO055
  Serial.print("Initializing BNO055... ");
  if(!bno.begin()) {
    bno = Adafruit_BNO055(55, 0x29);
    if(!bno.begin()) {
      Serial.println("FAILED!");
      Serial.println("ERROR:NO_BNO055");
    } else {
      Serial.println("OK (0x29)");
    }
  } else {
    Serial.println("OK (0x28)");
  }
  
  delay(1000);
  bno.setExtCrystalUse(true);
  
  // Initialize BMP390
  Serial.print("Initializing BMP390... ");
  if (!bmp.begin_I2C(0x77)) {
    if (!bmp.begin_I2C(0x76)) {
      Serial.println("FAILED!");
      Serial.println("ERROR:NO_BMP390");
    } else {
      Serial.println("OK (0x76)");
    }
  } else {
    Serial.println("OK (0x77)");
  }
  
  // Configure BMP390
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  
  // Initialize GPS
  Serial.print("Initializing GPS... ");
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("OK");
  
  // Initialize LoRa
  Serial.print("Initializing LoRa... ");
  loraSerial.begin(115200, SERIAL_8N1, LORA_RX, LORA_TX);
  delay(100);
  Serial.println("OK");
  
  // Run LoRa test and configuration
  delay(500);
  testLoRa();
  
  Serial.println("\nREADY");
  Serial.println("Streaming sensor data via Serial and LoRa...\n");
}

void loop() {
  // Update GPS data
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
  
  // Get BNO055 data
  sensors_event_t orientationData, angVelocityData, linearAccelData;
  sensors_event_t magnetometerData, accelerometerData, gravityData;
  
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
  
  imu::Quaternion quat = bno.getQuat();
  
  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  
  int8_t imu_temp = bno.getTemp();
  
  // Get BMP390 data
  float pressure = 0;
  float bmp_temp = 0;
  float altitude = 0;
  
  if (bmp.performReading()) {
    pressure = bmp.pressure / 100.0;
    bmp_temp = bmp.temperature;
    altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  }
  
  // Get GPS data
  float gps_lat = gps.location.isValid() ? gps.location.lat() : 0.0;
  float gps_lng = gps.location.isValid() ? gps.location.lng() : 0.0;
  float gps_alt = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
  float gps_speed = gps.speed.isValid() ? gps.speed.kmph() : 0.0;
  int gps_sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
  
  // Build FULL CSV data string for Serial output
  String csvData = "";
  csvData += String(orientationData.orientation.x, 2) + ",";
  csvData += String(orientationData.orientation.y, 2) + ",";
  csvData += String(orientationData.orientation.z, 2) + ",";
  csvData += String(angVelocityData.gyro.x, 2) + ",";
  csvData += String(angVelocityData.gyro.y, 2) + ",";
  csvData += String(angVelocityData.gyro.z, 2) + ",";
  csvData += String(accelerometerData.acceleration.x, 2) + ",";
  csvData += String(accelerometerData.acceleration.y, 2) + ",";
  csvData += String(accelerometerData.acceleration.z, 2) + ",";
  csvData += String(magnetometerData.magnetic.x, 1) + ",";
  csvData += String(magnetometerData.magnetic.y, 1) + ",";
  csvData += String(magnetometerData.magnetic.z, 1) + ",";
  csvData += String(linearAccelData.acceleration.x, 2) + ",";
  csvData += String(linearAccelData.acceleration.y, 2) + ",";
  csvData += String(linearAccelData.acceleration.z, 2) + ",";
  csvData += String(gravityData.acceleration.x, 2) + ",";
  csvData += String(gravityData.acceleration.y, 2) + ",";
  csvData += String(gravityData.acceleration.z, 2) + ",";
  csvData += String(sys) + ",";
  csvData += String(gyro) + ",";
  csvData += String(accel) + ",";
  csvData += String(mag) + ",";
  csvData += String(imu_temp) + ",";
  csvData += String(quat.w(), 4) + ",";
  csvData += String(quat.x(), 4) + ",";
  csvData += String(quat.y(), 4) + ",";
  csvData += String(quat.z(), 4) + ",";
  csvData += String(pressure, 2) + ",";
  csvData += String(bmp_temp, 2) + ",";
  csvData += String(altitude, 2) + ",";
  csvData += String(gps_lat, 6) + ",";
  csvData += String(gps_lng, 6) + ",";
  csvData += String(gps_alt, 2) + ",";
  csvData += String(gps_speed, 2) + ",";
  csvData += String(gps_sats);
  
  // Build COMPACT data for LoRa (only altimeter essentials)
  // Format: baro_alt,gps_alt,gps_lat,gps_lng,accel_z,speed,bmp_temp (~70 bytes)
  String loraData = "";
  loraData += String(altitude, 2) + ",";                       // Barometric altitude
  loraData += String(gps_alt, 2) + ",";                        // GPS altitude
  loraData += String(gps_lat, 6) + ",";                        // GPS Latitude
  loraData += String(gps_lng, 6) + ",";                        // GPS Longitude
  loraData += String(accelerometerData.acceleration.z, 2) + ","; // Vertical acceleration
  loraData += String(gps_speed, 2) + ",";                      // Speed
  loraData += String(bmp_temp, 2);                             // Temperature
  
  // Send FULL data to Serial (for direct USB connection)
  Serial.println(csvData);
  
  // Send COMPACT data via LoRa at specified interval
  unsigned long currentMillis = millis();
  if (currentMillis - lastLoraTransmit >= loraTransmitInterval) {
    sendLoraData(loraData);
    lastLoraTransmit = currentMillis;
  }
  
  delay(100);
}