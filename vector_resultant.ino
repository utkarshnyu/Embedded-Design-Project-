//code to do a vector resultant of VN at an angle of thetaN, weighted by lambdaN
#include <math.h>

float angles[10];
float V[10];
float lambda[10];



void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}


float magnitude(float lambda[], float V[], float angles[]){
  float x=0;
  float y=0;
  for(int i=0; i<10; i++){
    x += lambda[i]*V[i]*cosf(angles[i]);
    y += lambda[i]*V[i]*sinf(angles[i]);
  }
  return sqrt((x*x)+(y*y));

  
}

float angle(float lambda[], float V[], float angles[]){
  float x=0;
  float y=0;
  for(int i=0; i<10; i++){
    x += lambda[i]*V[i]*cosf(angles[i]);
    y += lambda[i]*V[i]*sinf(angles[i]);
  }
  return atanf(y/x);
}

