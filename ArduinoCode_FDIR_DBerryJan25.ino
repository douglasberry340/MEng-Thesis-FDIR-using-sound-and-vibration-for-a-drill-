#include <Audio.h>
#include <Adafruit_MSA301.h>  

//Microphone Setup
AudioInputI2S i2s1;               // I2S input from the SPH0645
AudioRecordQueue queue1;          // Queue for raw audio samples
AudioFilterStateVariable filter1;        
AudioAmplifier           amp1;          
AudioConnection          patchCord1(i2s1, 0, filter1, 0);
AudioConnection          patchCord2(filter1, 2, amp1, 0);
AudioConnection          patchCord3(amp1, queue1);

const int MIC_CALIBRATION_SAMPLES = 1500;  // Samples for dynamic offset calculation
int16_t dynamic_offset = 0;

//Accelerometer Setup
Adafruit_MSA301 msa;
const int ACCEL_SAMPLE_INTERVAL = 20;  // Accelerometer sampling interval in ms
unsigned long last_accel_time = 0;
float x_offset = 0.0, y_offset = 0.0, z_offset = 0.0;

const int SERIAL_BAUD_RATE = 115200; 

  //Microphone Initialisation 
  AudioMemory(150);  
  Serial.begin(SERIAL_BAUD_RATE);  
  filter1.frequency(100); // filter out DC & extremely low frequencies
  amp1.gain(1);        // amplify signal if required
  queue1.begin();  

  // Microphone Offset Calibration
  long offset_sum = 0;
  int count = 0;
  while (count < MIC_CALIBRATION_SAMPLES) {
    if (queue1.available() > 0) {
      int16_t *samples = queue1.readBuffer();
      for (int i = 0; i < 128 && count < MIC_CALIBRATION_SAMPLES; i++) {
        offset_sum += samples[i];
        count++;
      }
      queue1.freeBuffer();
    }
  }
  dynamic_offset = offset_sum / MIC_CALIBRATION_SAMPLES;
  Serial.print("Calculated Dynamic Offset: ");
  Serial.println(dynamic_offset);

  //Accelerometer Initialisation
  if (!msa.begin()) {
    Serial.println("Failed to initialise MSA301 accelerometer");
    while (1);  
  }

  msa.setPowerMode(MSA301_NORMALMODE);
  msa.setRange(MSA301_RANGE_16_G);
  msa.setDataRate(MSA301_DATARATE_250_HZ);
  msa.setBandwidth(MSA301_BANDWIDTH_125_HZ);

  // Accelerometer Offset Calibration
  float x_sum = 0.0, y_sum = 0.0, z_sum = 0.0;
  const int CALIBRATION_SAMPLES = 100;
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    msa.read();
    x_sum += msa.x;
    y_sum += msa.y;
    z_sum += msa.z;
    delay(10);
  }
  x_offset = x_sum / CALIBRATION_SAMPLES;
  y_offset = y_sum / CALIBRATION_SAMPLES;
  z_offset = z_sum / CALIBRATION_SAMPLES;
}

void loop() {
  unsigned long current_time = millis();

  // Microphone Data Processing
  if (queue1.available() > 0) {
    int16_t *samples = queue1.readBuffer();
    for (int i = 0; i < 128; i++) {
      int16_t corrected_sample = samples[i] - dynamic_offset;

      Serial.print("Sound,");
      Serial.println(corrected_sample);
    }
    queue1.freeBuffer();
  }

  //Accelerometer Data Processing
  if (current_time - last_accel_time >= ACCEL_SAMPLE_INTERVAL) {
    last_accel_time = current_time;

    // Read accelerometer data
    msa.read();
    float x_calibrated = msa.x - x_offset;
    float y_calibrated = msa.y - y_offset;
    float z_calibrated = msa.z - z_offset;

    Serial.print("Accel,");
    Serial.print(x_calibrated, 2);
    Serial.print(",");
    Serial.print(y_calibrated, 2);
    Serial.print(",");
    Serial.println(z_calibrated, 2);
  }
}
