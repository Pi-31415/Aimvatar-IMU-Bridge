#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Initialize the BNO055 sensor
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void setup(void) {
  // Start serial communication at 115200 baud rate
  Serial.begin(115200);
  while (!Serial) delay(10); // Wait for serial port to connect - needed for Leonardo only

  // Initialize the BNO055 sensor
  if (!bno.begin()) {
    Serial.println("No BNO055 detected");
    while (1);
  }
  delay(1000); // Delay for sensor to stabilize
}

void loop(void) {
  // Read sensor data
  sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
  
  // Print all sensor data in one line separated by commas
  printData(orientationData);
  printData(angVelocityData);
  printData(linearAccelData);
  printData(magnetometerData);
  printData(accelerometerData);
  printData(gravityData);

  // Print temperature
  int8_t boardTemp = bno.getTemp();
  Serial.print(boardTemp);
  
  // Print calibration data
  uint8_t system, gyro, accel, mag;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print(",");
  Serial.print(system);
  Serial.print(",");
  Serial.print(gyro);
  Serial.print(",");
  Serial.print(accel);
  Serial.print(",");
  Serial.print(mag);
  
  Serial.println(); // End of line
  delay(100); // Delay before next reading
}

// Function to print data for each sensor event
void printData(sensors_event_t event) {
  Serial.print(event.acceleration.x);
  Serial.print(",");
  Serial.print(event.acceleration.y);
  Serial.print(",");
  Serial.print(event.acceleration.z);
  Serial.print(",");
}