#include "arduinoFFT.h"

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
/*
These values can be changed in order to evaluate the functions
*/
#define CHANNEL A0
const uint16_t samples = 128; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 32000; //Hz, must be less than 10000 due to ADC

unsigned int sampling_period_us;
unsigned long microseconds;
double temp;

/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];


void setup()
{
  sampling_period_us = round(1000000*(1.0/samplingFrequency));
  Serial.begin(115200);
  Serial.println("Ready");
  analogReadResolution(12);
  analogReadAveraging(6);
}

void loop()
{
  /*SAMPLING*/
  for(int i=0; i<samples; i++)
  {
      microseconds = micros();    //Overflows after around 70 minutes!

      temp = analogRead(CHANNEL);
      vReal[i] =  ((temp * 3.3) / 2048) - 1.65;
      vImag[i] = 0;
      while(micros() < (microseconds + sampling_period_us)){
        //empty loop
      }
  }
  /* Print the results of the sampling according to time */
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
  Serial.println("Computed magnitudes:");
  PrintVector(vReal, (samples >> 1) );
  
  // while(1); /* Run Once */
  delay(1000); /* Repeat after delay */
}

void PrintVector(double *vData, uint16_t bufferSize)
{
  for (uint16_t i = 4; i < 41; i=i+4)
  {
    double abscissa;
        abscissa = ((i * 1.0 * samplingFrequency) / samples);
    
    Serial.print(abscissa, 6);
    Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}
