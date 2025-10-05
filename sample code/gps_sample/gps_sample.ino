#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

HardwareSerial GPSserial(1);  // Use UART1
TinyGPSPlus gps;

// GPS connections
#define GPS_TX_PIN 37  // GPS TX → ESP32 RX
#define GPS_RX_PIN 38  // GPS RX → ESP32 TX
#define GPS_INT_PIN 39 // optional (not required for reading data)

void setup() {
  Serial.begin(115200);
  GPSserial.begin(9600, SERIAL_8N1, GPS_TX_PIN, GPS_RX_PIN);  // GPS baud rate usually 9600
  pinMode(GPS_INT_PIN, INPUT);

  Serial.println("NEO-M8U GPS Reader Initialized");
  Serial.println("Waiting for GPS fix...");
}

void loop() {
  // Continuously read from GPS module
  while (GPSserial.available() > 0) {
    char c = GPSserial.read();
    gps.encode(c);  // Feed data to TinyGPS parser

    if (gps.location.isUpdated()) {
      Serial.println("=== GPS DATA ===");

      // Location
      Serial.print("Latitude: ");
      Serial.println(gps.location.lat(), 6);
      Serial.print("Longitude: ");
      Serial.println(gps.location.lng(), 6);
      Serial.print("Altitude: ");
      Serial.print(gps.altitude.meters());
      Serial.println(" m");

      // Date and Time
      if (gps.date.isValid()) {
        Serial.print("Date: ");
        Serial.print(gps.date.day());
        Serial.print("/");
        Serial.print(gps.date.month());
        Serial.print("/");
        Serial.println(gps.date.year());
      } else {
        Serial.println("Date: Not Available");
      }

      if (gps.time.isValid()) {
        Serial.print("Time (UTC): ");
        Serial.print(gps.time.hour());
        Serial.print(":");
        Serial.print(gps.time.minute());
        Serial.print(":");
        Serial.println(gps.time.second());
      } else {
        Serial.println("Time: Not Available");
      }

      // Optional Speed and Satellite Info
      Serial.print("Speed: ");
      Serial.print(gps.speed.kmph());
      Serial.println(" km/h");

      Serial.print("Satellites: ");
      Serial.println(gps.satellites.value());

      Serial.println("=================\n");
      delay(1000);
    }
  }
}
