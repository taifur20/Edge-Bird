#include <ESP_I2S.h>
I2SClass I2S;

#define GAIN 20.0   // Software gain (try 10–50)

// RMS calculation buffer
static float sumSq = 0;
static int count = 0;

void setup() {
  pinMode(45, OUTPUT);
  digitalWrite(45, HIGH);

  Serial.begin(115200);
  while (!Serial) { ; }

  // Setup I2S with PDM mic pins (BCLK=42, DOUT=41)
  I2S.setPinsPdmRx(42, 41);

  // Start I2S at 16 kHz, 16-bit, mono
  if (!I2S.begin(I2S_MODE_PDM_RX, 16000, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
    Serial.println("Failed to initialize I2S!");
    while (1);
  }

  Serial.println("I2S Microphone Started");
}

void loop() {
  int16_t sample = I2S.read();  // 16-bit signed sample

  if (sample != -1) {
    // Normalize to [-1, 1]
    float norm = (float)sample / 32768.0;

    // Apply software gain
    float amplified = norm * GAIN;

    // Accumulate RMS
    sumSq += amplified * amplified;
    count++;

    // Every 100 samples (~6ms @16kHz), compute RMS
    if (count >= 100) {
      float rms = sqrt(sumSq / count);

      // Scale for Serial Plotter visibility
      Serial.println(rms * 1000);

      // Reset buffer
      sumSq = 0;
      count = 0;
    }
  }
}
