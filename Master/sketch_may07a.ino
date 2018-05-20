#include "arduinoFFT.h"
#include <PWMServo.h>
#include "Ping2.h"

arduinoFFT FFT = arduinoFFT();
PWMServo myServo;
Ping2 ping = Ping2(20,21); 


#define CHANNEL A0         // for adc on pin 14 
const int samples = 128;   //This value MUST ALWAYS be a power of 2
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

//frequency mask
double fMask[10] = {1, 1, 1, 1, 1, 1, 2, 2, 2, 5};

//Car Wheel Pins
const int s1 =  9; //pin 21
const int s2 =  10; //pin 20

//parameters for distance detection/vector resultant
double lambda[10] = {2,4,0,0,0,0,0,0,10,0};
double V0 = 9;
double Tau = 0.038;

//Thresholds
int t = 10; //threshold for driveTime
int rotationTime = 1100/90;  // It takes 1200ms to rotate about 90deg at speed values of 212 and 162

//Final angle and distance of movement
double dist = 0;
double theta = 0;
double velocity = 0.1; // 10cm/s for speed-210; 15cm/s for speed-220; 35cm/s for speed 255
double driveTime = 0;

//ultrasonic
float dist_ult = 0;

//*******************************************************************************************
//*******************************************************************************************
void setup()
{
  ping = Ping2(20,21);                                    // Ping with trigger on pin 20, echo on pin 21 on teensy
  sampPeriod_us = round(1000000*(1.0/samplingFrequency));
  myServo.attach(3);                                          //pin 4 on teensy
  myServo.write(0);
  delay(500);
  Serial.begin(115200);
  analogReadResolution(12);
  analogReadAveraging(6);
  
  pinMode(s1, OUTPUT);                                       //pin 9 acts as output
  pinMode(s2, OUTPUT);                                       //pin 10 acts as output
  pinMode(13, OUTPUT);                                       //Led pin
}

//*******************************************************************************************
//*******************************************************************************************

void loop()
{
  //Servo motor turns and samples data + FFT + stores it in matrix                              
  servo_forward (5,1000);        // argument is the agle to swwep in each step like 5, 10 || second argument is the delay      
  myServo.write(0);
  delay(300);

  memset(beaconAngles, 0, sizeof(beaconAngles));
  memset(beaconDistances, 0, sizeof(beaconAngles));
  
  argMax(3); //Need to test and set a threshold
  for(int j = 0; j<10; j++)
  {
    Serial.print("Beacon Ang: ");
    Serial.print(beaconAngles[j]);
    Serial.print(", Dist: ");
    Serial.print(beaconDistances[j]);
    Serial.print(", Freq: ");
    Serial.println(5000+(j)*500);
  }
  //assign lambda weights here based on special cases
  
  //compute final angle and distance of movement
  dist = magnitude(lambda, beaconDistances, beaconAngles);
  theta = angle(lambda, beaconDistances, beaconAngles);
  Serial.print("theta,dist:");
  Serial.print(theta);
  Serial.print(",");
  Serial.println(dist);
 
  // Moving the car
  alignment(theta);                   // Takes the angle from the mic and aligns the car in that direction. Argument: angle in radians in the range of (-PI/2,PI/2)
  //ultrasonic sensor
  dist_ult = fire_sonic(5);                // argunemt is number of samples to average, result will be the range in cm 
  Serial.print("ultrasonic_dis: ");
  Serial.println(dist_ult);
  if(dist<=0.1 || dist_ult<=10)
  {
    if(theta != 0){
    alignment(0);   //rotate 90 deg 
    }
  }
  else
  {
    if(dist<dist_ult)
    {
     driveTime = t*dist/velocity;
     mov("Forward",driveTime,210,210);   // moves the car in "Forward","Reverse" or "Halt". 
     mov("Halt",20,190,190);           // Second argument is the time for delay function, it moves about 50cm for delay(5000) at speed_s1=210,speed_s2=200 (3rd and 4th arguments)
     //write code for what happens next
    }
    else 
    {
     driveTime = t*(dist_ult-20)/velocity;
     mov("Forward",driveTime,210,210);   // moves the car in "Forward","Reverse" or "Halt". 
     mov("Halt",20,190,190);           // Second argument is the time for delay function, it moves about 50cm for delay(5000) at speed_s1=210,speed_s2=200 (3rd and 4th arguments)
     //write code for what happens next
    }
  }
}

//*******************************************************************************************
//*******************************************************************************************


void StoreFFTData(double *data, int servoDegree)
{
  int desiredFreq = 0;
  for(uint16_t j = 20; j < 39; j=j+2) //for 1k->10k, j=4, j<41, j+=4; for 5k->10k: j=20; j<39, j+=2;
  {
    FFTMatrix[desiredFreq][servoDegree] = data[j]; //FFTMatrix is a global variable that stores values of all the FFTs for each freq band for each servo motor angle
    desiredFreq++;
  }
}


//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************

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

//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************

void argMax(double thresh)
{
 double currMax = 0;
  for(int i = 0; i < 10; i++)
  {
    currMax = 0;
    for(int j = 0; j <= 36; j++)
    {
      if((FFTMatrix[i][j]>currMax) & (fMask[i]*FFTMatrix[i][j]>thresh))
      {
         currMax = FFTMatrix[i][j];
         beaconAngles[i] = (j*PI/36); //radians
         //beaconAngles[i] = j*5; //degrees
         beaconDistances[i] = amp2dist(currMax, V0, Tau);
      }
    }
  }
}

//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************

int angmap(double angle)
{
  int angle_deg = angle*180/PI;                 //converting radians to degrees and take the abs value(because of (-PI/2,PI/2))
  Serial.println("in angmap");
  Serial.println(angle_deg);
  int d[91];
  memset(d, 0, sizeof(d));

  for(int k =0;k<=90;k++)
  {
    d[k] = rotationTime*k;                              
  }
  return d[90-abs(angle_deg)];
}

//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************

void alignment(double beaconAngle)
{ 
  Serial.println(beaconAngle);

  if(beaconAngle>=0)
  {
    //car has to turn left
    analogWrite(s1,210);                // s1 moves forward
    analogWrite(s2,162);               //s2 moves backward (s2 terminals are inverted)
    delay(angmap(beaconAngle));
    analogWrite(s1,190);
    analogWrite(s2,190);
    delay(1000);
  }
  else if(beaconAngle<0)
  {
    //car has to turn right
    analogWrite(s1,162);                // s1 moves backwards
    analogWrite(s2,210);                // s2 moves forward (s2 terminals are inverted)
    delay(angmap(beaconAngle));
    analogWrite(s1,190);
    analogWrite(s2,190);
    delay(1000);
  }
}

//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************


void mov(String s,double del,int speed_s1,int speed_s2)
{
 if(s.equals("Forward"))
 {
  analogWrite(s1,speed_s1);  // 200-255 s1 rotates CW with highest speed at 255
  analogWrite(s2,speed_s2);  // 200-255 s2 rotates in the same direction as s1 above since we have inverted the terminals
  delay(del);
 }
 else if(s.equals("Reverse"))
 {
  analogWrite(s1,speed_s1);  // 128-180 s1 rotates CCW with highest speed at 128
  analogWrite(s2,speed_s2);  // 128-180 s2 rotates in the same direction as s1 above since we have inverted the terminals
  delay(del);
 }
 else if(s.equals("Halt"))
 {
  analogWrite(s1,speed_s1);  //180-190 s1 stops rotating
  analogWrite(s2,speed_s2);  //180-190 s2 stops rotating
  delay(del);
  }
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************

double magnitude(double lambda[], double V[], double angles[])
{
  double x=0;
  double y=0;
  for(int i=0; i<10; i++)
   {
    x += lambda[i]*V[i]*cos(angles[i]);
    y += lambda[i]*V[i]*sin(angles[i]);
   }
  return sqrt((x*x)+(y*y));
}

//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************

double angle(double lambda[], double V[], double angles[])
{
  double x=0;
  double y=0;
  for(int i=0; i<10; i++)
   {
    x += lambda[i]*V[i]*cos(angles[i]);
    y += lambda[i]*V[i]*sin(angles[i]);
   }

  if (x == 0 && y == 0)
   {
    return 0;
   }
  else 
   {
   return atan(y/x); 
   }
}

//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************



double amp2dist(double A, double V0, double Tau)           // returning distance characteristic from amplitude
{
  return log(A/V0)/(-Tau);
}

//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************

float fire_sonic(int res)                                  // for sonic sensor HC-SRO4
{                              
  float di = 0;
  for (int x = 0; x <res;x++ )
   {  
     ping.fire();
     // Serial.print("Microseconds: ");
     // Serial.print(ping.microseconds());
     // Serial.print(" | Inches ");
     // Serial.print(ping.inches());
     // Serial.print(" | Centimeters: ");
     // Serial.println( ping.centimeters() );
     // Serial.println();
     di += ping.centimeters();
     
   }
  return (di/res);
}

//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************

void servo_forward (int step, int del)                     // function for servo forward + FFT
{

    for(int i = 0;  i <= 180; i+=step)
     {         
      myServo.write(i);
      delay(del);
      SampleData(vReal, vImag, samples, sampPeriod_us);
      FFT.Compute(vReal, vImag, samples, FFT_FORWARD);     // Compute FFT 
      FFT.ComplexToMagnitude(vReal, vImag, samples);       // Compute magnitudes 
      StoreFFTData(vReal, i/step);
     }

}

//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************

void servo_backward (int step, int del)                    // figure out the indexing while coming back **************
{

    for(int i = 180;  i >= 0; i-=step)
     {
      myServo.write(i);
      delay(del);
      SampleData(vReal, vImag, samples, sampPeriod_us);
      FFT.Compute(vReal, vImag, samples, FFT_FORWARD);     // Compute FFT */
      FFT.ComplexToMagnitude(vReal, vImag, samples);       // Compute magnitudes */
      StoreFFTData(vReal, i/step);
     }

}

//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
