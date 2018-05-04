//code to do both the filtering and the angle assignment
//Try blocking the output to increase efficiency
#include <math.h>
#include <arduinoFFT.h>


const uint16_t nfft = 256; //This value MUST ALWAYS be a power of 2

const int N = 100; //length of filter, and buffer
const double fs = 50000; //sampling frequency
const int L = nfft;
const double TMic = 1; //period of the microphone's oscillation, assume integer multiple of fs/L
const int fmLength = nfft;//int(TMic*fs/L); //filter max vector length for each filtering period
const double filterFreq = fs/L;

//baseband filter coefficients
const double P[N] = {   1.0244210E-03, 1.0566031E-03, 1.1136336E-03, 1.1967055E-03, 1.3068667E-03, 1.4450103E-03, 1.6118649E-03, 1.8079871E-03, 2.0337543E-03, 2.2893588E-03, 2.5748033E-03, 2.8898975E-03, 3.2342561E-03, 3.6072978E-03, 4.0082460E-03, 4.4361308E-03, 4.8897919E-03, 5.3678834E-03, 5.8688794E-03, 6.3910814E-03, 6.9326264E-03, 7.4914967E-03, 8.0655302E-03, 8.6524324E-03, 9.2497889E-03, 9.8550794E-03, 1.0465691E-02, 1.1078936E-02, 1.1692065E-02, 1.2302282E-02, 1.2906767E-02, 1.3502688E-02, 1.4087217E-02, 1.4657553E-02, 1.5210935E-02, 1.5744659E-02, 1.6256098E-02, 1.6742714E-02, 1.7202077E-02, 1.7631881E-02, 1.8029955E-02, 1.8394279E-02, 1.8722996E-02, 1.9014426E-02, 1.9267073E-02, 1.9479638E-02, 1.9651024E-02, 1.9780345E-02, 1.9866933E-02, 1.9910340E-02, 1.9910340E-02, 1.9866933E-02, 1.9780345E-02, 1.9651024E-02, 1.9479638E-02, 1.9267073E-02, 1.9014426E-02, 1.8722996E-02, 1.8394279E-02, 1.8029955E-02, 1.7631881E-02, 1.7202077E-02, 1.6742714E-02, 1.6256098E-02, 1.5744659E-02, 1.5210935E-02, 1.4657553E-02, 1.4087217E-02, 1.3502688E-02, 1.2906767E-02, 1.2302282E-02, 1.1692065E-02, 1.1078936E-02, 1.0465691E-02, 9.8550794E-03, 9.2497889E-03, 8.6524324E-03, 8.0655302E-03, 7.4914967E-03, 6.9326264E-03, 6.3910814E-03, 5.8688794E-03, 5.3678834E-03, 4.8897919E-03, 4.4361308E-03, 4.0082460E-03, 3.6072978E-03, 3.2342561E-03, 2.8898975E-03, 2.5748033E-03, 2.2893588E-03, 2.0337543E-03, 1.8079871E-03, 1.6118649E-03, 1.4450103E-03, 1.3068667E-03, 1.1967055E-03, 1.1136336E-03, 1.0566031E-03, 1.0244210E-03   };
//cutoff frequencies for the cosine modulation
const double FC[10] = {  2*1000/fs,   2*2000/fs,  2*3000/fs,  2*4000/fs,   2*5000/fs,  2*6000/fs,  2*7000/fs,  2*8000/fs,   2*9000/fs,  2*10000/fs  }; 

long dt = 1/fs, ts1, ts2;
uint16_t value;
float volt;

float T = 0; //threshold
float angles[10];

float x; //input from mic
double XBuff[nfft] ; //circular buffer utilized by the filter function, static used so all values automatically initialized to zero
int xWrite = 0;
int yWrite = 0;

int argMaxVals[10];

arduinoFFT FFT = arduinoFFT();

//filter bank FFT
double fBank[10][nfft];

double filtMax[10][fmLength];
//[i][j] is the output value of the ith filter at the jth fft block

double vReal[nfft];
double vImag[nfft];

void setup() {
  Serial.begin(9600);
  
  //Set FFT of entire filter bank in fBank struct
  for(int fInd=0; fInd<10; fInd++){
    for(int j=0; j<nfft; j++){
      vReal[j] = 0;
      vImag[j] = 0;
    }
    //Serial.println(vReal[150]);
    for(int j=0; j<N; j++){
      vReal[j] = 2*P[j]*cos(PI*FC[fInd]*j);//cosine modulate it
    }
    //FFT.Windowing(PBandR, nfft, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, nfft, FFT_FORWARD); /* Compute FFT */
    FFT.ComplexToMagnitude(vReal, vImag, nfft);
    for(int j=0; j<nfft; j++){
      fBank[fInd][j] = vReal[j]; //assign filter bank to magnitude values. This is a real filter so it should be fine
    }
    //double a = PBandR[0];
    //Serial.println(a);
    

  }
  Serial.println(' ');
  //adc setup
  ADC0_CFG1 |= _BV(5);                                              // DOV CLOCK BY 2
  ADC0_CFG1 |= _BV(3) | _BV(2);                                     // 16 BIT MODE 
  ADC0_CFG2 |= _BV(4)| _BV(2);                                      // MUX SEL ADC B + High speed conversion | _BV(2) 
  
  ADC0_SC3 |=  ADC_SC3_ADCO | ADC_SC3_AVGE  | _BV(1);       // 32 SAMPLE AVG | _BV(0)
  
  ADC0_SC1A = 5;                                                    // ad5 inpu, diff = 0 (WHY DOES THE ORDER MATTER )  +  enable interrupt _BV(6) | XX

  ts1 = 0;
  


}

void loop() {
  ts2 = micros();
  if (ts2-ts1 >= dt){
  value = ADC0_RA;
  volt = ((float)value/65535.0)*3.3-1.65;
  ts1 = micros();
  XBuff[xWrite] = volt;
  //Serial.println(XBuff[xWrite]);

  xWrite++;
  if(!(xWrite%nfft)){
      filter(XBuff, yWrite);
      yWrite++;
      yWrite %= fmLength;
      Serial.println(filtMax[9][0]);
    }
  }
  xWrite %= nfft;
}


void filter(double XBuff[], int outInd){
  //fft of xBuff
  double XFFT[nfft];
  double vReal[nfft] = {*XBuff};
  static double vImag[nfft];
  //FFT.Windowing(XBuff, nfft, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, nfft, FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(vReal, vImag, nfft);
  for(int j=0; j<nfft; j++){
    XFFT[j] = vReal[j]; //assign filter bank to magnitude values. This is a real filter so it should be fine
    
  }
  //Serial.println(maxval(XFFT, T, nfft));
  

  for(int i=0; i<10; i++){
    double out[nfft];
    for(int sumInd=0; sumInd<nfft; sumInd++){
       out[sumInd] = fBank[i][sumInd]*XFFT[sumInd];
    }
    filtMax[i][outInd] = maxval(out, T, nfft);
    
  }
}
  //will filter and return max value for each subband of the filter bank




void argmax(float x[10][L],int L){ //returns row-wise argmax value of 2-D array x, using threshold T for the min value
  int maxInd = 0;
  
  for(int j = 0; j<10; j++){
    float currMax = 0;
    for(int i = 0; i<L; i++){
      if (x[j][i]>currMax){
        currMax = x[j][i];
        maxInd = i;
      }
    }
    if(currMax==0){
      argMaxVals[j] = L+1; //return something nonsensical, i.e. an angle greater than 2pi, if the signal is not detected
    }
    else{
      argMaxVals[j] = maxInd;  
    }
  }
}


float maxval(double x[], float T, int L){ //return maximum value, used to keep track of max over multiple filtering periods.
  float currMax = 0;
  for(int i = 0; i<L; i++){
    if ((x[i]>currMax)&(x[i]>T)){
      currMax = x[i];
    }
  }
  return currMax;
}


