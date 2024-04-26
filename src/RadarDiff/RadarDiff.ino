//Normalize Ouput

#include <arduinoFFT.h>
#include <TimerOne.h>

// Define the ADC midpoint (512 is the default)
#define ADC_MIDPOINT 512

// Define the sample rate (8800 S/s is the default for 16 MHz Boards)
#define SAMPLE_RATE 8800

//Diff
unsigned long last_millis = 0; // keep track of the last time we read the ADC
const uint16_t sampleSize = 512;
const uint8_t  ind_thresh = 128;
const uint8_t  indSize = 16;
float prev = 0;
volatile int i = 0;
volatile float samples[sampleSize] = {};
// volatile int j = 0;
// volatile float samples_B[sampleSize] = {};
volatile short sample;
uint16_t peak_ind_idx = 0;
bool dataReady = false;
bool bufferReady = false;
bool fftPrint = true;

/* Create FFT object */
const float samplingFrequency = (float) SAMPLE_RATE;

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

typedef union {
  float floatingPoint;
  byte binary[4];
} binaryFloat;

void setup() {
  Serial.begin(115200); // Use a faster baud rate so we can print faster
  while(!Serial);

  Timer1.initialize(113.63636363636); //Sample Rate 8800hz, 113.6363us
  Timer1.attachInterrupt(get_sample);

  //read_differential_start();
  read_differential_setup(); // configure the ADC
  
  last_millis = millis(); // record a starting timestamp
}

void loop() {
  if (bufferReady) {
    // for(int j = 0; j < sampleSize; j++) {
    //   Serial.println(samples[j]);
    // }
    float vImag[sampleSize] = {};
    ArduinoFFT<float> FFT = ArduinoFFT<float>(samples, vImag, sampleSize, samplingFrequency);
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);	/* Weigh data */
    FFT.compute(FFTDirection::Forward); /* Compute FFT */
    FFT.complexToMagnitude(); /* Compute magnitudes */
    
    float maxInd;
    float maxVal;
    int dcRange = 1;
    // Remove DC
    for (int i = 0; i < dcRange; i++) {
      samples[i] = 0;
    }
    // samples[8] = 0;
    // Serial.println(samples[1]);

    // for(int j = 0; j < ind_thresh; j++) {
    //   Serial.print(j);
    //   Serial.print(' ');
    //   Serial.println(samples[j]);
    // }
    
    PrintVector(samples, ind_thresh, SCL_INDEX, &maxInd, &maxVal);
    

    float valThresh = 400;
    float valMin = 0;
    volatile float peak_ind[indSize] = {};
    if (maxVal > valThresh) {
    //if (maxVal < valThresh && maxVal > valMin) {
      peak_ind[peak_ind_idx++] = maxInd;
    } else {
      peak_ind[peak_ind_idx++] = 0;
    }

    if (peak_ind_idx == indSize) {
      peak_ind_idx = 0;
    }

    float cumul = 0;
    uint8_t valid_samples = 0;
    for (uint8_t i = 0; i < indSize; i++) {
      if (peak_ind[i] != 0) {
        cumul += (float)peak_ind[i];
        valid_samples++;
      }
    }

    float max_freq = 0;
    if (valid_samples != 0) {
      max_freq = cumul/valid_samples;
    } else {
      max_freq = 0;
    }
    max_freq = max_freq * samplingFrequency / sampleSize;
    float speed = ((uint16_t)max_freq)*(0.0125/2)*(3600/1000);
    binaryFloat send;
    send.floatingPoint = speed;
    Serial.write('\n');
    Serial.write(send.binary, 4);
    // Serial.print(maxInd);
    // Serial.print(' ');
    // Serial.print(maxVal);
    // Serial.print(' ');
    // Serial.print(max_freq);
    // Serial.print(' ');
    // Serial.print(speed);
    // Serial.print('\n');
    // if (speed < 40) {
    //   Serial.println((prev + speed)/2);
    // }
    bufferReady = false;
  }
}

// Sets up the ADC to run in differential mode
void read_differential_setup() {
  // choose 1.1V for reference ADC voltage
  ADMUX = (1<<REFS1);
  // enable ADC, ADC frequency=16MB/128=125kHz (set of prescaler), enable interrupts
  ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADIE ); 
  ADCSRB = 0x00;
}

// Tell ADC to start taking one sample. This will return immediately while the ADC keeps running
void read_differential_start() {
  // set MUX5:0 to 001001. Positive Differential Input => ADC0 and 
  // Negative Differential Input => ADC1 with Gain 10x.
  ADMUX |= (1<<MUX3) | (1<<MUX0) | (1<<MUX1);
  // start conversion
  ADCSRA |= (1<<ADSC);            
}

// This checks if the ADC has finished taking the most recent sample.
// only needed when polling, not when using an interrupt
bool read_differential_status() {
  return (ADCSRA & (1<<ADSC));     
}

// Fetches whatever the most recent 
short read_differential_value() {
  uint8_t low = ADCL;
  uint8_t high = ADCH;
  if(high & (1<<1)){
      // in differential mode our value is between -512 to 511 (not 0 to 1023). 
      // it means we have 9 bits and 10th bit is the sign bit. but because 
      // the number of ADCH and ADCL bits are 10, for signed number
      // we don't have repetition of 1 in "ADCH" byte.
    high |= 0b11111110;           
  }                               

  // arrange (ADCH ADCL) to a 16 bit and return the value.
  return ((short) high << 8) | low;      
}

void get_sample(void) {
  //Serial.println("Get_Sample");
  read_differential_start(); // Reads the value from the ADC
  //last_millis = millis();        // Record the time we're starting this sample
  //Serial.println(dataReady);
}

float PrintVector(float *vData, uint16_t bufferSize, uint8_t scaleType, float *m_ind, float *m_val)
{
  float max = 0;
  float m_abscissa = 0;
  int ind_start = 0;
  for (uint16_t i = ind_start; i < bufferSize; i++)
  {
    float abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
	break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);
	break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / sampleSize);
	break;
    }
    if (vData[i] > max) {
    //if (vData[i] > max && vData[i] > 60) {
      max = vData[i];
      m_abscissa = abscissa;
    }
    // Serial.print(abscissa, 6);
    // if(scaleType==SCL_FREQUENCY)
    //   Serial.print("Hz");
    // Serial.print(" ");
    // // Serial.print(i == ind_start ? 2000 : 0);
    // // Serial.print(" ");
    // Serial.println(vData[i], 4);
  }
  // Serial.println(max);
  // Serial.print(" ");
  *m_val = max;
  *m_ind = m_abscissa;
}
int temp = 0;
// ADC Interrupt Service Routine (ISR)
ISR(ADC_vect) {
  // in this example we are polling in the loop(), so nothing to do here
  // we don't need read_differential_status() in the ISR because we know it is done
  
  if (bufferReady == 0) {
    samples[i++] = read_differential_value();
  }
  if (i == sampleSize) {
    bufferReady = 1;
  }
  if (i >= sampleSize) {
    i = 0;
  }
}
