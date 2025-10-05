/* 
   ESP32-S3 + IM69D130 + Edge Impulse
   Bird Sound Recognition
   ---------------------------------
   Uses verified I2S PDM RX setup (CLK=42, DATA=41)
   and feeds audio samples into Edge Impulse classifier.
*/

#include <bird_sound_recognition_inferencing.h>
#include <ESP_I2S.h>

I2SClass I2S;

/** Audio inference struct */
typedef struct {
    int16_t *buffer;
    uint8_t buf_ready;
    uint32_t buf_count;
    uint32_t n_samples;
} inference_t;

static inference_t inference;
static signed short sampleBuffer[2048];  // temp buffer
static bool debug_nn = false;            // set true for DSP debug output
static bool record_status = true;

/**
 * @brief Arduino setup function
 */
void setup() {
    pinMode(45, OUTPUT);   // mic VDD enable pin
    digitalWrite(45, HIGH);
    delay(100);

    Serial.begin(115200);
    while (!Serial);

    Serial.println("ESP32-S3 IM69D130 Edge Impulse Bird Recognition");

    // show EI model info
    ei_printf("Inferencing settings:\n");
    ei_printf("\tInterval: %d ms\n", EI_CLASSIFIER_INTERVAL_MS);
    ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    ei_printf("\tSample length: %d ms\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT / (EI_CLASSIFIER_FREQUENCY / 1000));
    ei_printf("\tNo. of classes: %d\n", EI_CLASSIFIER_LABEL_COUNT);

    if (!microphone_inference_start(EI_CLASSIFIER_RAW_SAMPLE_COUNT)) {
        ei_printf("ERR: Failed to setup audio buffer\n");
        while (1);
    }

    ei_printf("Recording...\n");
}

/**
 * @brief Arduino main loop
 */
void loop() {
    if (!microphone_inference_record()) {
        ei_printf("ERR: Failed to record audio\n");
        return;
    }

    signal_t signal;
    signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
    signal.get_data = &microphone_audio_signal_get_data;
    ei_impulse_result_t result = { 0 };

    EI_IMPULSE_ERROR r = run_classifier(&signal, &result, debug_nn);
    if (r != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", r);
        return;
    }

    // print predictions
    ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.):\n",
              result.timing.dsp, result.timing.classification, result.timing.anomaly);

    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("  %s: ", result.classification[ix].label);
        ei_printf_float(result.classification[ix].value);
        ei_printf("\n");
    }

#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("  anomaly score: ");
    ei_printf_float(result.anomaly);
    ei_printf("\n");
#endif
}

/* -------------------- AUDIO FUNCTIONS -------------------- */

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
    const int32_t samples_to_read = (uint32_t)arg / 2; // 16-bit samples

    while (record_status) {
        for (int i = 0; i < samples_to_read; i++) {
            int32_t s = I2S.read();
            if (s != -1) {
                sampleBuffer[i] = (int16_t)(s * 8); // amplify a bit
            } else {
                sampleBuffer[i] = 0;
            }
        }
        audio_inference_callback(samples_to_read * 2); // pass bytes
    }
    vTaskDelete(NULL);
}

static bool microphone_inference_start(uint32_t n_samples) {
    inference.buffer = (int16_t *)malloc(n_samples * sizeof(int16_t));
    if (!inference.buffer) return false;

    inference.buf_count = 0;
    inference.n_samples = n_samples;
    inference.buf_ready = 0;

    // Initialize PDM RX with working pins
    I2S.setPinsPdmRx(42, 41); // CLK = 42, DATA = 41
    if (!I2S.begin(I2S_MODE_PDM_RX, EI_CLASSIFIER_FREQUENCY,
                   I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
        ei_printf("ERR: Failed to initialize I2S in PDM RX mode\n");
        return false;
    }

    record_status = true;
    xTaskCreate(capture_samples, "CaptureSamples", 32 * 1024,
                (void*)sizeof(sampleBuffer), 10, NULL);

    return true;
}

static bool microphone_inference_record(void) {
    while (inference.buf_ready == 0) {
        delay(1);
    }
    inference.buf_ready = 0;
    return true;
}

static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr) {
    numpy::int16_to_float(&inference.buffer[offset], out_ptr, length);
    return 0;
}

static void microphone_inference_end(void) {
    I2S.end();
    free(inference.buffer);
}
