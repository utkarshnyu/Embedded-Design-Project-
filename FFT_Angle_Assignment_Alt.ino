#include "arduinoFFT.h"

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
/*
These values can be changed in order to evaluate the functions
*/
#define CHANNEL A0
const int samples = 128; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 32000; //Hz, must be less than 10000 due to ADC

long sampling_period_us;

/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];

double FFTMatrix[10][180]; //rows are the desired frequencies (1k, 2k, 3k ... 10k), columns contain fft magnitude for each servo motor degree

//Return values of the 2-D argmax
int beaconAngle = 0;
double ampAtBeaconAngle = 0;
int freqBeaconAngle = 0; //0 = 1khz, 1 = 2khz,...,9 = 10kHz

void setup()
{
  sampling_period_us = round(1000000*(1.0/samplingFrequency));
  Serial.begin(115200);
  analogReadResolution(12);
  analogReadAveraging(6);
}

void loop()
{
  //Servo motor turns and samples data + FFT + stores it in matrix
  for(int i = 0;  i < 180; i++)
  {
    //insert servo motor moving code here
    SampleData(vReal, vImag, samples, sampling_period_us);
    FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
    FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
    StoreFFTData(vReal, i);
  }
  argMax(5); //Need to test and set a threshold
  Serial.print("Beacon Ang: ");
  Serial.print(beaconAngle);
  Serial.print(", Amp: ");
  Serial.print(ampAtBeaconAngle);
  Serial.print(", Freq: ");
  Serial.println((freqBeaconAngle+1)*1000);
  /*
  //printing out the FFT results for each frequency (1k, 2k,...,10k) for only one of the servo degrees... there will be 180 readings like this
  for(int j = 0; j < 180; j++)
  {
    Serial.print(FFTMatrix[0][j]);
    Serial.print(",");
  }
  Serial.println(" ");*/
}

void StoreFFTData(double *data, int servoDegree)
{
  int desiredFreq = 0;
  for(uint16_t i = 4; i < 41; i=i+4)
  {
    FFTMatrix[desiredFreq][servoDegree] = data[i]; //FFTMatrix is a global variable that stores 
    desiredFreq++;
  }
}

void SampleData(double realData[], double imagData[], int numSamples, long sampPeriod)
{
  double temp;
  unsigned long timeStamp;
  for(int i=0; i<samples; i++)
  {
      timeStamp = micros();
      temp = analogRead(CHANNEL);
      realData[i] =  ((temp * 3.3) / 2048) - 1.65;
      imagData[i] = 0;
      while(micros() < (timeStamp + sampPeriod));
  }
}

void argMax(double thresh)
{
  double currMax = 0;
  
  for(int i = 0; i < 10; i++)
  {
    for(int j = 0; j < 180; j++)
    {
      if((FFTMatrix[i][j]>currMax) & (FFTMatrix[i][j]>thresh))
      {
        ampAtBeaconAngle = FFTMatrix[i][j];
        beaconAngle = j;
        freqBeaconAngle = i;
      }
    }
  }
}

