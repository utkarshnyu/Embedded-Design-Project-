#include "arduinoFFT.h"
#include <PWMServo.h>
arduinoFFT FFT = arduinoFFT();
PWMServo myServo;
/*
These values can be changed in order to evaluate the functions
*/
#define CHANNEL A0
const int samples = 128; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 32000;
long sampPeriod_us;
/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];
double FFTMatrix[10][18]; //rows are the desired frequencies (1k, 2k, 3k ... 10k), columns contain fft magnitude for each servo motor degree
//Return values of the 2-D argmax
double beaconAngles[10];
double beaconDistances[10];
double ampAtBeaconAngle = 0;
int freqBeaconAngle = 0; //0 = 1khz, 1 = 2khz,...,9 = 10kHz
void setup()
{
 sampPeriod_us = round(1000000*(1.0/samplingFrequency));
 myServo.attach(SERVO_PIN_A);
 myServo.write(0);
 delay(500);
 Serial.begin(115200);
 analogReadResolution(12);
 analogReadAveraging(6);
}
void loop()
{
 //Servo motor turns and samples data + FFT + stores it in matrix
 for(int i = 0;  i < 180; i+=10)
 {
   //insert servo motor moving code here
   myServo.write(i);
   delay(91);
   SampleData(vReal, vImag, samples, sampPeriod_us);
   FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
   FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
   StoreFFTData(vReal, i/10);
 }
 myServo.write(0);
 delay(300);
 beaconAngle = 0;
 ampAtBeaconAngle = 0;
 freqBeaconAngle = 0;
 argMax(5); //Need to test and set a threshold
 Serial.print("Beacon Ang: ");
 Serial.print(beaconAngle);
 Serial.print(", Amp: ");
 Serial.print(ampAtBeaconAngle);
 Serial.print(", Freq: ");
 Serial.println((freqBeaconAngle+1)*1000);
 /*
 //printing out the FFT results for each frequency (1k, 2k,...,10k) for only one of the servo degrees... there will be 180 readings like this
 for(int j = 0; j < 10; j++)
 {
   Serial.print(FFTMatrix[j][0]);
   Serial.print(",");
 }
 Serial.println(" ");*/
}
void StoreFFTData(double *data, int servoDegree)
{
 int desiredFreq = 0;
 for(uint16_t j = 4; j < 41; j=j+4)
 {
   FFTMatrix[desiredFreq][servoDegree] = data[j]; //FFTMatrix is a global variable that stores values of all the FFTs for each freq band for each servo motor angle
   desiredFreq++;
 }
}
void SampleData(double realData[], double imagData[], int numSamples, long sampPeriod)
{
 double adcData;
 unsigned long timeStamp;
 for(int i=0; i<samples; i++)
 {
     timeStamp = micros();
     adcData = analogRead(CHANNEL);
     realData[i] =  ((adcData * 3.3) / 2048) - 1.65;
     imagData[i] = 0;
     while(micros() < (timeStamp + sampPeriod));
 }
}
void argMax(double thresh)
{
 
 
 for(int i = 0; i < 10; i++)
 {
  double currMax = 0;
   for(int j = 0; j <= 18; j++)
   {
     if((FFTMatrix[i][j]>currMax) & (FFTMatrix[i][j]>thresh))
     {
      currMax = FFTMatrix[i][j];
      beaconAngles[i] = j*PI/18;
      //beaconDistances[i] = 
     }
   }
 }
}
