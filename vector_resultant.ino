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


double magnitude(double lambda[], double V[], double angles[]){
  double x=0;
  double y=0;
  for(int i=0; i<10; i++){
    x += lambda[i]*V[i]*cos(angles[i]);
    y += lambda[i]*V[i]*sin(angles[i]);
  }
  return sqrt((x*x)+(y*y));

  
}

double angle(double lambda[], double V[], double angles[]){
  double x=0;
  double y=0;
  for(int i=0; i<10; i++){
    x += lambda[i]*V[i]*cos(angles[i]);
    y += lambda[i]*V[i]*sin(angles[i]);
  }
  return atanf(y/x);
}

