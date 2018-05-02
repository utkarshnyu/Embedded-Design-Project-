//Functions for assigning lambda for the weighted vector sum operation
//assuming filter function has been completed and returns 10 vectors {y0...y9} of the past values of y;
#include <math.h>


const int N = 100;
const int L = N;
const float fs = 50000;
const float TMic = 1; //period of the microphone's oscillation, assume integer multiple of fs/L
const int fmLength = int(TMic*fs/L); //filter max vector length for each filtering period
const float filterFreq = fs/L;
float T = 0.1; //threshold
int maxInd = 0;
float angles[10];

//assume output of each filter in filter bank is held in struct YOUT
typedef struct filterOutput{
  float y_0[L];
  float y_1[L];
  float y_2[L];
  float y_3[L];
  float y_4[L];
  float y_5[L];
  float y_6[L];
  float y_7[L];
  float y_8[L];
  float y_9[L];
}fOut;

fOut YOUT;

//structure to hold the results of each filtering period maximum for each filter
typedef struct filterPeriodOutput{
  float filtMax_0[fmLength];
  float filtMax_1[fmLength];
  float filtMax_2[fmLength];
  float filtMax_3[fmLength];
  float filtMax_4[fmLength];
  float filtMax_5[fmLength];
  float filtMax_6[fmLength];
  float filtMax_7[fmLength];
  float filtMax_8[fmLength];
  float filtMax_9[fmLength];
}filtMax;

filtMax FILTMAX;




void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  //IF STATEMENT FOR FULL XWRITE REACHING ITS END
  FILTMAX.filtMax_0[maxInd] = maxval(YOUT.y_0, T, L); //this assignment will happen every time the filter buffer maxes
  FILTMAX.filtMax_1[maxInd] = maxval(YOUT.y_0, T, L);
  FILTMAX.filtMax_2[maxInd] = maxval(YOUT.y_0, T, L);
  FILTMAX.filtMax_3[maxInd] = maxval(YOUT.y_0, T, L);
  FILTMAX.filtMax_4[maxInd] = maxval(YOUT.y_0, T, L);
  FILTMAX.filtMax_5[maxInd] = maxval(YOUT.y_0, T, L);
  FILTMAX.filtMax_6[maxInd] = maxval(YOUT.y_0, T, L);
  FILTMAX.filtMax_7[maxInd] = maxval(YOUT.y_0, T, L);
  FILTMAX.filtMax_8[maxInd] = maxval(YOUT.y_0, T, L);
  FILTMAX.filtMax_9[maxInd] = maxval(YOUT.y_0, T, L);

  //calculate angle in radians of the detected maximum
  angles[0] = 2 * PI * argmax(FILTMAX.filtMax_0, TMic*fs/L) / (TMic*fs/L);
  angles[1] = 2 * PI * argmax(FILTMAX.filtMax_1, TMic*fs/L) / (TMic*fs/L);
  angles[2] = 2 * PI * argmax(FILTMAX.filtMax_2, TMic*fs/L) / (TMic*fs/L);
  angles[3] = 2 * PI * argmax(FILTMAX.filtMax_3, TMic*fs/L) / (TMic*fs/L);
  angles[4] = 2 * PI * argmax(FILTMAX.filtMax_4, TMic*fs/L) / (TMic*fs/L);
  angles[5] = 2 * PI * argmax(FILTMAX.filtMax_5, TMic*fs/L) / (TMic*fs/L);
  angles[6] = 2 * PI * argmax(FILTMAX.filtMax_6, TMic*fs/L) / (TMic*fs/L);
  angles[7] = 2 * PI * argmax(FILTMAX.filtMax_7, TMic*fs/L) / (TMic*fs/L);
  angles[8] = 2 * PI * argmax(FILTMAX.filtMax_8, TMic*fs/L) / (TMic*fs/L);
  angles[9] = 2 * PI * argmax(FILTMAX.filtMax_9, TMic*fs/L) / (TMic*fs/L);
  

  
  maxInd++;
}

int argmax(float x[],int L){ //returns argmax value of array x, using threshold T for the min value
  int maxInd = 0;
  float currMax = 0;
  for(int i = 0; i<L; i++){
    if (x[i]>currMax){
      currMax = x[i];
      maxInd = i;
    }
  }
  if(currMax==0){
    return TMic*fs/L+1; //return something nonsensical, i.e. an angle greater than 2pi, if the signal is not detected
  }
  else{
    return maxInd;
  } 
}


float maxval(float x[], float T, int L){ //return maximum value, used to keep track of max over multiple filtering periods.
  float currMax = 0;
  for(int i = 0; i<L; i++){
    if ((x[i]>currMax)&(x[i]>T)){
      currMax = x[i];
    }
  }
  return currMax;
}

