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
double FFTMatrix[10][37]; //rows are the desired frequencies (1k, 2k, 3k ... 10k), columns contain fft magnitude for each servo motor degree

//Return values of the 2-D argmax
double beaconAngles[10];
double beaconDistances[10];

//Car Wheel Pins
const int s1 =  9; //pin 21
const int s2 =  10; //pin 20

void setup()
{
  sampPeriod_us = round(1000000*(1.0/samplingFrequency));
  myServo.attach(3);
  myServo.write(0);
  delay(500);
  Serial.begin(115200);
  analogReadResolution(12);
  analogReadAveraging(6);
  pinMode(s1, OUTPUT); //pin 21 acts as output
  pinMode(s2, OUTPUT); //pin 20 acts as output
  pinMode(13, OUTPUT);
}

void loop()
{
  //Servo motor turns and samples data + FFT + stores it in matrix
  for(int i = 0;  i <= 180; i+=5)
  {
    //insert servo motor moving code here
    myServo.write(i);
    delay(200);
    SampleData(vReal, vImag, samples, sampPeriod_us);
    FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
    FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
    StoreFFTData(vReal, i/10);
  }
  myServo.write(0);
  delay(300);
  memset(beaconAngles, 0, sizeof(beaconAngles));
  memset(beaconDistances, 0, sizeof(beaconAngles));
  argMax(2); //Need to test and set a threshold
  for(int j = 0; j<10; j++)
  {
    Serial.print("Beacon Ang: ");
    Serial.print(beaconAngles[j]);
    Serial.print(", Amp: ");
    Serial.print(beaconDistances[j]);
    Serial.print(", Freq: ");
    Serial.println((j+1)*1000);
  }
  /*digitalWrite(13, HIGH);
  mov("Forward",2000,210,210);
  alignment(0);
  mov("Halt",2000,190,190);
  digitalWrite(13, LOW);*/
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
 double currMax = 0;

  for(int i = 0; i < 10; i++)
  {
    currMax = 0;
    for(int j = 0; j <= 36; j++)
    {
      if((FFTMatrix[i][j]>currMax) & (FFTMatrix[i][j]>thresh))
      {
         currMax = FFTMatrix[i][j];
         //beaconAngles[i] = j*PI/18;
         beaconAngles[i] = j*10;
         beaconDistances[i] = currMax;
      }
    }
  }
}

void mov(String s,int del,int speed_s1,int speed_s2)
{
 if(s.equals("Forward")){
    analogWrite(s1,speed_s1);   // 200-255 s1 rotates CW with highest speed at 255
    analogWrite(s2,speed_s2);   // 128-180 s2 rotates CCW with highest speed at 128
    delay(del);
 }
 else if(s.equals("Reverse")){
   analogWrite(s1,speed_s1);   // 128-180 s1 rotates CCW with highest speed at 128
   analogWrite(s2,speed_s2);   // 200-255 s2 rotates CW with highest speed at 255
   delay(del);
 }
 else if(s.equals("Halt")){
   analogWrite(s1,speed_s1);   //180-190 s1 stops rotating
   analogWrite(s2,speed_s2);   //180-190 s2 stops rotating
   delay(del);
 }
}

void alignment(int beaconAngle)
{
  if(beaconAngle<90){
    //car has to turn left
    analogWrite(s1,210);
    analogWrite(s2,172);
    delay(angmap(90-beaconAngle));
    analogWrite(s1,190);
    analogWrite(s2,190);
  }
  else if(beaconAngle>90){
    //car has to turn right
    analogWrite(s1,172);
    analogWrite(s2,210);
    delay(angmap(beaconAngle-90));
    analogWrite(s1,190);
    analogWrite(s2,190);
  }
}

int angmap(int angle)
{
  int d[91];
  for(int k =0;k<=90;k++){
    d[k] = 500*(k/90);
  }
  return d[angle];
}

