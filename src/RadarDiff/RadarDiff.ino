#include <arduinoFFT.h>
#include <TimerOne.h>

// Define the ADC midpoint (512 is the default)
#define ADC_MIDPOINT 256

// Define the sample rate (8800 S/s is the default for 16 MHz Boards)
#define SAMPLE_RATE 8800

//Diff
unsigned long last_millis = 0; // keep track of the last time we read the ADC
const uint16_t sampleSize = 512;
volatile int i = 0;
volatile double samples[sampleSize] = {};
volatile short sample;
bool dataReady = false;
bool bufferReady = false;

//FFT
const double samplingFrequency = 8800;
double vImag[sampleSize];

ArduinoFFT<double> FFT = ArduinoFFT<double>(samples, vImag, sampleSize, samplingFrequency);

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

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
    for(int j = 0; j < sampleSize; j++) {
      //Serial.println(samples[j]);
      vImag[j] = 0.0; 
    }
    bufferReady = false;
    /* Print the results of the simulated sampling according to time */
    Serial.println("Data:");
    PrintVector(samples, sampleSize, SCL_TIME);
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);	/* Weigh data */
    Serial.println("Weighed data:");
    PrintVector(samples, sampleSize, SCL_TIME);
    FFT.compute(FFTDirection::Forward); /* Compute FFT */
    Serial.println("Computed Real values:");
    PrintVector(samples, sampleSize, SCL_INDEX);
    Serial.println("Computed Imaginary values:");
    PrintVector(vImag, sampleSize, SCL_INDEX);
    FFT.complexToMagnitude(); /* Compute magnitudes */
    Serial.println("Computed magnitudes:");
    PrintVector(samples, (sampleSize >> 1), SCL_FREQUENCY);
    double x = FFT.majorPeak();
    Serial.println(x, 6);
  }
  delay(100);
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
  ADMUX |= (1<<MUX3) | (1<<MUX0);// | (1<<MUX1);
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
  dataReady = 1;
  //Serial.println(dataReady);
}

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
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
    Serial.print(abscissa, 6);
    if(scaleType==SCL_FREQUENCY)
      Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}

// ADC Interrupt Service Routine (ISR)
ISR(ADC_vect) {
  // in this example we are polling in the loop(), so nothing to do here
  // we don't need read_differential_status() in the ISR because we know it is done
  if(dataReady) {
    //Serial.println(i);
    dataReady = false;
    samples[i++] = read_differential_value();
    i == sampleSize ? bufferReady = 1 : bufferReady = 0;
    i > sampleSize ? i = 0 :i=i ;
  }
}
