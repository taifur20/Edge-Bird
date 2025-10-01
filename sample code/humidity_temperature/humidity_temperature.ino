#include <Wire.h>

#define HS3003_ADDR 0x44   // I2C address of HS3003 (0x44 or 0x45 depending on wiring)

void setup() {
  Serial.begin(115200);
  pinMode(45, OUTPUT);
  digitalWrite(45, HIGH);
  Wire.begin(); // default SDA=8, SCL=9 for ESP32-S3 (adjust if needed)
  delay(100);
}

void loop() {
  // Request 4 bytes from HS3003
  Wire.requestFrom(HS3003_ADDR, 4);
  if (Wire.available() == 4) {
    uint16_t raw_humidity = (Wire.read() << 8) | Wire.read();
    uint16_t raw_temp     = (Wire.read() << 8) | Wire.read();

    // Convert to actual values (see HS3003 datasheet)
    float humidity = ((raw_humidity & 0x3FFF) * 100.0) / 16383.0;
    float temperature = (((raw_temp >> 2) & 0x3FFF) * 165.0 / 16383.0) - 40.0;

    Serial.print("Temperature: ");
    Serial.print(temperature, 2);
    Serial.print(" °C, Humidity: ");
    Serial.print(humidity, 2);
    Serial.println(" %RH");
  } else {
    Serial.println("Failed to read from HS3003!");
  }

  delay(2000); // read every 2 seconds
}
