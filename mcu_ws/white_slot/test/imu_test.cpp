#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define I2C_SDA 21
#define I2C_SCL 22

// TwoWire I2CBNO = TwoWire(0);

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);  // Default I2C address

void setup() {
  Serial.begin(115200);
  Wire.begin(21,22);
  // I2CBNO.begin(I2C_SDA, I2C_SCL);


  // Initialize BNO055 sensor
  if (!bno.begin()) {
    Serial.println("BNO055 not detected.");
    while (1);
  }
  
  delay(1000);
  bno.setExtCrystalUse(true);  // Use external crystal
}


void loop() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float roll = euler.z();   // Z
  float pitch = euler.y();  // Y
  float yaw = euler.x();    // X

  // Send data in CSV format
  Serial.print(roll); Serial.print(",");
  Serial.print(pitch); Serial.print(",");
  Serial.println(yaw);

  delay(50);
}