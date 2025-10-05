/***********************************************************************
   ESP32-S3 Bird Environment Logger + GPS
   -------------------------------------------------------------
   • HS3003 – Temperature & Humidity
   • LPS22HB – Pressure
   • BMM350 – Magnetometer
   • MAX17048 – Battery Fuel Gauge
   • IM69D130 – Mic for Bird Sound (Edge Impulse model)
   • NEO-M8U GPS (TX=37, RX=38)
   • SD card – Data logging every 30 s
 ***********************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "DFRobot_BMM350.h"
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>
#include <ESP_I2S.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <bird_sound_recognition_inferencing.h>   // Edge Impulse model header

// =========================================================
// Pin definitions
// =========================================================
#define POWER_PIN 45
#define SDA_PIN   8
#define SCL_PIN   9
#define SD_CS     10
#define SD_MOSI   11
#define SD_MISO   13
#define SD_SCK    12
#define GPS_TX_PIN 37  // GPS TX → ESP32 RX
#define GPS_RX_PIN 38  // GPS RX → ESP32 TX

SPIClass spiSD(HSPI);
HardwareSerial GPSserial(1);
TinyGPSPlus gps;

// =========================================================
// Sensor addresses
// =========================================================
#define HS3003_ADDR  0x44
#define LPS22HB_ADDR 0x5C
#define WHO_AM_I     0x0F
#define CTRL_REG1    0x10
#define PRESS_OUT_XL 0x28
#define TEMP_OUT_L   0x2B

// =========================================================
// Instances
// =========================================================
DFRobot_BMM350_I2C bmm350(&Wire, I2C_ADDRESS);
SFE_MAX1704X lipo(MAX1704X_MAX17048);
I2SClass I2S;

// =========================================================
// Inference buffer
// =========================================================
typedef struct {
  int16_t *buffer;
  uint8_t buf_ready;
  uint32_t buf_count;
  uint32_t n_samples;
} inference_t;
static inference_t inference;
static signed short sampleBuffer[2048];
static bool record_status = true;
static bool debug_nn = false;

// =========================================================
// Timing
// =========================================================
unsigned long lastLogTime = 0;
const unsigned long logInterval = 30000; // 30 s

// =========================================================
// --- SENSOR READ FUNCTIONS ---
// =========================================================
bool readHS3003(float &t, float &h) {
  Wire.requestFrom(HS3003_ADDR, 4);
  if (Wire.available() == 4) {
    uint16_t rh = (Wire.read() << 8) | Wire.read();
    uint16_t rt = (Wire.read() << 8) | Wire.read();
    h = ((rh & 0x3FFF) * 100.0) / 16383.0;
    t = (((rt >> 2) & 0x3FFF) * 165.0 / 16383.0) - 40.0;
    return true;
  }
  return false;
}

bool readLPS22HB(float &p) {
  Wire.beginTransmission(LPS22HB_ADDR);
  Wire.write(PRESS_OUT_XL | 0x80);
  Wire.endTransmission();
  Wire.requestFrom(LPS22HB_ADDR, 3);
  if (Wire.available() == 3) {
    int32_t raw = Wire.read();
    raw |= (Wire.read() << 8);
    raw |= (Wire.read() << 16);
    if (raw & 0x00800000) raw |= 0xFF000000;
    p = raw / 4096.0;
    return true;
  }
  return false;
}

bool readBMM350(float &mx, float &my, float &mz) {
  sBmm350MagData_t mag = bmm350.getGeomagneticData();
  mx = mag.x; my = mag.y; mz = mag.z;
  return true;
}

bool readBattery(float &v, float &soc) {
  v = lipo.getVoltage();
  soc = lipo.getSOC();
  return true;
}

// =========================================================
// --- AUDIO CAPTURE & INFERENCE ---
// =========================================================
static void audio_inference_callback(uint32_t n_bytes) {
  for (int i = 0; i < n_bytes >> 1; i++) {
    inference.buffer[inference.buf_count++] = sampleBuffer[i];
    if (inference.buf_count >= inference.n_samples) {
      inference.buf_count = 0;
      inference.buf_ready = 1;
    }
  }
}

static void capture_samples(void* arg) {
  const int32_t samples_to_read = (uint32_t)arg / 2;
  while (record_status) {
    for (int i = 0; i < samples_to_read; i++) {
      int32_t s = I2S.read();
      if (s != -1) sampleBuffer[i] = (int16_t)(s * 8);
      else sampleBuffer[i] = 0;
    }
    audio_inference_callback(samples_to_read * 2);
  }
  vTaskDelete(NULL);
}

static bool microphone_inference_start(uint32_t n_samples) {
  inference.buffer = (int16_t*)malloc(n_samples * sizeof(int16_t));
  if (!inference.buffer) return false;
  inference.buf_count = 0;
  inference.n_samples = n_samples;
  inference.buf_ready = 0;
  I2S.setPinsPdmRx(42, 41);
  if (!I2S.begin(I2S_MODE_PDM_RX, EI_CLASSIFIER_FREQUENCY,
                 I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
    Serial.println("ERR: I2S init failed!");
    return false;
  }
  record_status = true;
  xTaskCreate(capture_samples, "CaptureSamples", 32 * 1024,
              (void*)sizeof(sampleBuffer), 10, NULL);
  return true;
}

static bool microphone_inference_record(void) {
  while (inference.buf_ready == 0) delay(1);
  inference.buf_ready = 0;
  return true;
}

static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr) {
  numpy::int16_to_float(&inference.buffer[offset], out_ptr, length);
  return 0;
}

String runBirdInference() {
  if (!microphone_inference_record()) return "no_audio";

  signal_t signal;
  signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
  signal.get_data = &microphone_audio_signal_get_data;
  ei_impulse_result_t result = {0};

  EI_IMPULSE_ERROR r = run_classifier(&signal, &result, debug_nn);
  if (r != EI_IMPULSE_OK) {
    Serial.printf("Classifier error %d\n", r);
    return "error";
  }

  float best_val = 0;
  String best_label = "unknown";
  for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    if (result.classification[i].value > best_val) {
      best_val = result.classification[i].value;
      best_label = result.classification[i].label;
    }
  }
  Serial.printf("Detected bird: %s (%.2f)\n", best_label.c_str(), best_val);
  return best_label;
}

// =========================================================
// --- GPS FUNCTIONS ---
// =========================================================
bool readGPS(double &lat, double &lon, int &sat, String &dateStr, String &timeStr) {
  while (GPSserial.available() > 0) {
    char c = GPSserial.read();
    gps.encode(c);
  }

  if (gps.location.isValid()) {
    lat = gps.location.lat();
    lon = gps.location.lng();
    sat = gps.satellites.value();

    if (gps.date.isValid()) {
      char buf[16];
      sprintf(buf, "%02d/%02d/%04d", gps.date.day(), gps.date.month(), gps.date.year());
      dateStr = String(buf);
    } else dateStr = "N/A";

    if (gps.time.isValid()) {
      char buf[16];
      sprintf(buf, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
      timeStr = String(buf);
    } else timeStr = "N/A";

    return true;
  }
  return false;
}

// =========================================================
// Setup
// =========================================================
void setup() {
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, HIGH);
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  // GPS init
  GPSserial.begin(9600, SERIAL_8N1, GPS_TX_PIN, GPS_RX_PIN);
  Serial.println("Initializing GPS...");
  delay(2000);

  // SD init
  spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, spiSD)) {
    Serial.println("SD card init failed!"); while (1);
  }
  File f = SD.open("/data_log.csv", FILE_APPEND);
  if (f && f.size() == 0)
    f.println("Date,Time,Latitude,Longitude,Sat,Temp(C),Humidity(%),Pressure(hPa),MagX(uT),MagY(uT),MagZ(uT),Battery(V),Battery(%),Bird");
  f.close();

  // Sensors init
  Wire.beginTransmission(LPS22HB_ADDR);
  Wire.write(CTRL_REG1); Wire.write(0x10); Wire.endTransmission();
  if (bmm350.begin()) { Serial.println("BMM350 fail!"); while (1); }
  bmm350.setOperationMode(eBmm350NormalMode);
  bmm350.setPresetMode(BMM350_PRESETMODE_HIGHACCURACY,BMM350_DATA_RATE_25HZ);
  bmm350.setMeasurementXYZ();
  if (!lipo.begin()) { Serial.println("MAX17048 fail!"); while (1); }
  lipo.quickStart(); lipo.setThreshold(20);

  // Mic init
  if (!microphone_inference_start(EI_CLASSIFIER_RAW_SAMPLE_COUNT)) {
    Serial.println("Mic init failed!"); while (1);
  }

  Serial.println("System ready. Logging every 30 s...");
}

// =========================================================
// Loop
// =========================================================
void loop() {
  if (millis() - lastLogTime < logInterval) return;
  lastLogTime = millis();

  float t=0,h=0,p=0,mx=0,my=0,mz=0,bv=0,bs=0;
  double lat=0, lon=0;
  int sat=0;
  String dateStr="N/A", timeStr="N/A";

  readHS3003(t,h);
  readLPS22HB(p);
  readBMM350(mx,my,mz);
  readBattery(bv,bs);
  readGPS(lat,lon,sat,dateStr,timeStr);

  String bird = runBirdInference();

  Serial.println("---- New Record ----");
  Serial.printf("%s %s | Lat: %.6f, Lon: %.6f (%d sat)\n", dateStr.c_str(), timeStr.c_str(), lat, lon, sat);
  Serial.printf("T=%.2fC  H=%.2f%%  P=%.2fhPa\n", t,h,p);
  Serial.printf("Mag: %.2f, %.2f, %.2f  Bat: %.2fV %.2f%%\n", mx,my,mz,bv,bs);
  Serial.println(bird);

  File df = SD.open("/data_log.csv", FILE_APPEND);
  if (df) {
    df.print(dateStr); df.print(",");
    df.print(timeStr); df.print(",");
    df.print(lat,6); df.print(",");
    df.print(lon,6); df.print(",");
    df.print(sat); df.print(",");
    df.print(t,2); df.print(",");
    df.print(h,2); df.print(",");
    df.print(p,2); df.print(",");
    df.print(mx,2); df.print(",");
    df.print(my,2); df.print(",");
    df.print(mz,2); df.print(",");
    df.print(bv,2); df.print(",");
    df.print(bs,2); df.print(",");
    df.println(bird);
    df.close();
    Serial.println("Data saved.\n");
  } else {
    Serial.println("SD write error!");
  }
}
