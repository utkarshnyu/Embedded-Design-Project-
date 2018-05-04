#include <math.h>
#include <kinetis.h>


const int N = 100; //length of filter, and buffer
const float fs = 50000; //sampling frequency

long dt = 1/fs, ts1, ts2;
uint16_t value;

const int L = 10*N;
//baseband filter coefficients
const float P[100] = {   1.0244210E-03, 1.0566031E-03, 1.1136336E-03, 1.1967055E-03, 1.3068667E-03, 1.4450103E-03, 1.6118649E-03, 1.8079871E-03, 2.0337543E-03, 2.2893588E-03, 2.5748033E-03, 2.8898975E-03, 3.2342561E-03, 3.6072978E-03, 4.0082460E-03, 4.4361308E-03, 4.8897919E-03, 5.3678834E-03, 5.8688794E-03, 6.3910814E-03, 6.9326264E-03, 7.4914967E-03, 8.0655302E-03, 8.6524324E-03, 9.2497889E-03, 9.8550794E-03, 1.0465691E-02, 1.1078936E-02, 1.1692065E-02, 1.2302282E-02, 1.2906767E-02, 1.3502688E-02, 1.4087217E-02, 1.4657553E-02, 1.5210935E-02, 1.5744659E-02, 1.6256098E-02, 1.6742714E-02, 1.7202077E-02, 1.7631881E-02, 1.8029955E-02, 1.8394279E-02, 1.8722996E-02, 1.9014426E-02, 1.9267073E-02, 1.9479638E-02, 1.9651024E-02, 1.9780345E-02, 1.9866933E-02, 1.9910340E-02, 1.9910340E-02, 1.9866933E-02, 1.9780345E-02, 1.9651024E-02, 1.9479638E-02, 1.9267073E-02, 1.9014426E-02, 1.8722996E-02, 1.8394279E-02, 1.8029955E-02, 1.7631881E-02, 1.7202077E-02, 1.6742714E-02, 1.6256098E-02, 1.5744659E-02, 1.5210935E-02, 1.4657553E-02, 1.4087217E-02, 1.3502688E-02, 1.2906767E-02, 1.2302282E-02, 1.1692065E-02, 1.1078936E-02, 1.0465691E-02, 9.8550794E-03, 9.2497889E-03, 8.6524324E-03, 8.0655302E-03, 7.4914967E-03, 6.9326264E-03, 6.3910814E-03, 5.8688794E-03, 5.3678834E-03, 4.8897919E-03, 4.4361308E-03, 4.0082460E-03, 3.6072978E-03, 3.2342561E-03, 2.8898975E-03, 2.5748033E-03, 2.2893588E-03, 2.0337543E-03, 1.8079871E-03, 1.6118649E-03, 1.4450103E-03, 1.3068667E-03, 1.1967055E-03, 1.1136336E-03, 1.0566031E-03, 1.0244210E-03   };
//cutoff frequencies for the cosine modulation
const float FC[10] = {  2*1000/fs,   2*2000/fs,  2*3000/fs,  2*4000/fs,   2*5000/fs,  2*6000/fs,  2*7000/fs,  2*8000/fs,   2*9000/fs,  2*10000/fs  }; 
float x; //input from mic
float y_0[L]; //array of output values for each filter
static float XBuff[N] ; //circular buffer utilized by the filter function, static used so all values automatically initialized to zero
int xWrite = 0; //write index for the circular buffer, easier to keep track of than pointer to element in XBuff;
int yWrite = 0;


void setup() {
  Serial.begin(230400);
  ADC0_CFG1 |= _BV(5);                                              // DOV CLOCK BY 2
  ADC0_CFG1 |= _BV(3) | _BV(2);                                     // 16 BIT MODE 
  ADC0_CFG2 |= _BV(4)| _BV(2);                                      // MUX SEL ADC B + High speed conversion | _BV(2) 
  
  ADC0_SC3 |=  ADC_SC3_ADCO | ADC_SC3_AVGE  | _BV(1);       // 32 SAMPLE AVG | _BV(0)
  
  ADC0_SC1A = 5;                                                    // ad5 inpu, diff = 0 (WHY DOES THE ORDER MATTER )  +  enable interrupt _BV(6) | XX

  ts1 = 0;
}

void loop() {
  //example of how filter will be used 
  //assuming mic input for current time is held in *x
  //would be assigned on sample ready interrupt normally
  ts2 = micros();
  if (ts2-ts1 >= dt){
    value = ADC0_RA;
    XBuff[xWrite] = ((float)value/65535.0)*3.3;
    ts1 = micros();
  }
  
  y_0[yWrite] = filter(XBuff, P, FC[0], xWrite);//FC[index] is the frequency that you want to filter for, the fractions correspond to [1000 Hz 2000 Hz....10000 Hz]
  Serial.println(XBuff[xWrite]);
  yWrite++;
  yWrite %= L;
  xWrite++;
  xWrite %= 100;
}

float filter(float XBuff[], const float P[], float fc, int writeInd){
  float out = 0; //output for current time
  for(int convInd=0; convInd<N; convInd++){ //perform the convolution
    int revInd = (N-convInd)%N;
    int readInd = (convInd+writeInd)%N;
    out += 2 * P[revInd] * XBuff[readInd] * cosf(PI * fc * float(revInd)); //weird indexing on P is because of how circular buffer is constructed s.t. [... x[-1] x[0] x[-99] ...]
  }
  return out;
}


