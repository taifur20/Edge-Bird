#include <Wire.h>

#define LPS22HB_ADDR 0x5C  // Default I2C address (0x5C or 0x5D depending on SDO pin)

// LPS22HB registers
#define WHO_AM_I      0x0F
#define CTRL_REG1     0x10
#define PRESS_OUT_XL  0x28
#define TEMP_OUT_L    0x2B

void setup() {
  pinMode(45, OUTPUT);
  digitalWrite(45, HIGH);
  Serial.begin(115200);
  Wire.begin(); // SDA/SCL default (ESP32-S3 usually SDA=8, SCL=9)

  // Check device ID
  Wire.beginTransmission(LPS22HB_ADDR);
  Wire.write(WHO_AM_I);
  Wire.endTransmission();
  Wire.requestFrom(LPS22HB_ADDR, 1);
  if (Wire.available()) {
    uint8_t id = Wire.read();
    if (id == 0xB1) {
      Serial.println("LPS22HB detected!");
    } else {
      Serial.print("Unexpected WHO_AM_I: 0x");
      Serial.println(id, HEX);
    }
  }

  // Configure CTRL_REG1: ODR = 1 Hz (output data rate)
  Wire.beginTransmission(LPS22HB_ADDR);
  Wire.write(CTRL_REG1);
  Wire.write(0x10); // 00010000 -> ODR = 1 Hz
  Wire.endTransmission();
}

void loop() {
  // Read pressure (3 bytes: XL, L, H)
  Wire.beginTransmission(LPS22HB_ADDR);
  Wire.write(PRESS_OUT_XL | 0x80); // Auto-increment address
  Wire.endTransmission();
  Wire.requestFrom(LPS22HB_ADDR, 3);
  int32_t rawPress = 0;
  if (Wire.available() == 3) {
    rawPress = Wire.read();
    rawPress |= (Wire.read() << 8);
    rawPress |= (Wire.read() << 16);
    if (rawPress & 0x00800000) { // sign extend 24-bit to 32-bit
      rawPress |= 0xFF000000;
    }
  }
  float pressure_hPa = rawPress / 4096.0; // Conversion per datasheet

  // Read temperature (2 bytes: L, H)
  Wire.beginTransmission(LPS22HB_ADDR);
  Wire.write(TEMP_OUT_L | 0x80);
  Wire.endTransmission();
  Wire.requestFrom(LPS22HB_ADDR, 2);
  int16_t rawTemp = 0;
  if (Wire.available() == 2) {
    rawTemp = Wire.read();
    rawTemp |= (Wire.read() << 8);
  }
  float temperature_C = rawTemp / 100.0; // Conversion per datasheet

  // Print results
  Serial.print("Pressure: ");
  Serial.print(pressure_hPa, 2);
  Serial.print(" hPa, Temperature: ");
  Serial.print(temperature_C, 2);
  Serial.println(" °C");

  delay(1000);
}
