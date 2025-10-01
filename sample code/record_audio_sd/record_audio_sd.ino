#include <ESP_I2S.h>
#include <SD.h>
#include <SPI.h>

I2SClass I2S;

// SD card pins (adjust for your wiring)
#define SD_CS   10
#define SD_MOSI 11
#define SD_MISO 13
#define SD_SCK  12

SPIClass spiSD(HSPI);

#define SAMPLE_RATE   16000     // 16kHz
#define BITS_PER_SAMPLE 16
#define RECORD_TIME   20        // seconds
#define FILENAME      "/record.wav"

File audioFile;

uint32_t numSamples = SAMPLE_RATE * RECORD_TIME;

void writeWavHeader(File file, uint32_t numSamples, uint32_t sampleRate, uint16_t bitsPerSample) {
  uint32_t dataSize = numSamples * (bitsPerSample / 8);

  // RIFF header
  file.write((const uint8_t*)"RIFF", 4);
  uint32_t chunkSize = 36 + dataSize;
  file.write((uint8_t*)&chunkSize, 4);
  file.write((const uint8_t*)"WAVE", 4);

  // fmt subchunk
  file.write((const uint8_t*)"fmt ", 4);
  uint32_t subChunk1Size = 16;
  uint16_t audioFormat = 1; // PCM
  uint16_t numChannels = 1; // Mono
  uint32_t byteRate = sampleRate * numChannels * bitsPerSample / 8;
  uint16_t blockAlign = numChannels * bitsPerSample / 8;

  file.write((uint8_t*)&subChunk1Size, 4);
  file.write((uint8_t*)&audioFormat, 2);
  file.write((uint8_t*)&numChannels, 2);
  file.write((uint8_t*)&sampleRate, 4);
  file.write((uint8_t*)&byteRate, 4);
  file.write((uint8_t*)&blockAlign, 2);
  file.write((uint8_t*)&bitsPerSample, 2);

  // data subchunk
  file.write((const uint8_t*)"data", 4);
  file.write((uint8_t*)&dataSize, 4);
}

void setup() {
  pinMode(45, OUTPUT);
  digitalWrite(45, HIGH);

  Serial.begin(115200);
  while (!Serial) {;}

  // I2S setup
  I2S.setPinsPdmRx(42, 41);
  if (!I2S.begin(I2S_MODE_PDM_RX, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
    Serial.println("Failed to initialize I2S!");
    while (1);
  }

  // Init custom SPI for SD card
  spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, spiSD)) {
    Serial.println("SD Card init failed!");
    while (1);
  }

  Serial.println("SD Card initialized.");

  // Create file
  audioFile = SD.open(FILENAME, FILE_WRITE);
  if (!audioFile) {
    Serial.println("Failed to open file!");
    while (1);
  }

  // Write placeholder WAV header
  writeWavHeader(audioFile, numSamples, SAMPLE_RATE, BITS_PER_SAMPLE);

  Serial.println("Recording started...");
  
  // Record samples
  for (uint32_t i = 0; i < numSamples; i++) {
    int16_t sample = I2S.read();
    if (sample != -1) {
      audioFile.write((uint8_t*)&sample, 2);
    } else {
      i--; // retry if no sample
    }
  }

  Serial.println("Recording finished!");

  // Fix header with correct size
  audioFile.seek(0);
  writeWavHeader(audioFile, numSamples, SAMPLE_RATE, BITS_PER_SAMPLE);

  audioFile.close();
  Serial.println("WAV file saved to SD card!");
}

void loop() {
  // Nothing, recording only once
}
