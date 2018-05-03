#include <math.h>
#include <ADC.h>

const uint8_t adc_pin = A6;
ADC adc;
volatile uint16_t adc_val = 0;

const int N = 100; //length of filter, and buffer
const float fs = 50000; //sampling frequency
const int L = N;
//baseband filter coefficients
const float P[N] = {   1.0244210E-03, 1.0566031E-03, 1.1136336E-03, 1.1967055E-03, 1.3068667E-03, 1.4450103E-03, 1.6118649E-03, 1.8079871E-03, 2.0337543E-03, 2.2893588E-03, 2.5748033E-03, 2.8898975E-03, 3.2342561E-03, 3.6072978E-03, 4.0082460E-03, 4.4361308E-03, 4.8897919E-03, 5.3678834E-03, 5.8688794E-03, 6.3910814E-03, 6.9326264E-03, 7.4914967E-03, 8.0655302E-03, 8.6524324E-03, 9.2497889E-03, 9.8550794E-03, 1.0465691E-02, 1.1078936E-02, 1.1692065E-02, 1.2302282E-02, 1.2906767E-02, 1.3502688E-02, 1.4087217E-02, 1.4657553E-02, 1.5210935E-02, 1.5744659E-02, 1.6256098E-02, 1.6742714E-02, 1.7202077E-02, 1.7631881E-02, 1.8029955E-02, 1.8394279E-02, 1.8722996E-02, 1.9014426E-02, 1.9267073E-02, 1.9479638E-02, 1.9651024E-02, 1.9780345E-02, 1.9866933E-02, 1.9910340E-02, 1.9910340E-02, 1.9866933E-02, 1.9780345E-02, 1.9651024E-02, 1.9479638E-02, 1.9267073E-02, 1.9014426E-02, 1.8722996E-02, 1.8394279E-02, 1.8029955E-02, 1.7631881E-02, 1.7202077E-02, 1.6742714E-02, 1.6256098E-02, 1.5744659E-02, 1.5210935E-02, 1.4657553E-02, 1.4087217E-02, 1.3502688E-02, 1.2906767E-02, 1.2302282E-02, 1.1692065E-02, 1.1078936E-02, 1.0465691E-02, 9.8550794E-03, 9.2497889E-03, 8.6524324E-03, 8.0655302E-03, 7.4914967E-03, 6.9326264E-03, 6.3910814E-03, 5.8688794E-03, 5.3678834E-03, 4.8897919E-03, 4.4361308E-03, 4.0082460E-03, 3.6072978E-03, 3.2342561E-03, 2.8898975E-03, 2.5748033E-03, 2.2893588E-03, 2.0337543E-03, 1.8079871E-03, 1.6118649E-03, 1.4450103E-03, 1.3068667E-03, 1.1967055E-03, 1.1136336E-03, 1.0566031E-03, 1.0244210E-03   };
//cutoff frequencies for the cosine modulation
const float FC[10] = {  2*1000/fs,   2*2000/fs,  2*3000/fs,  2*4000/fs,   2*5000/fs,  2*6000/fs,  2*7000/fs,  2*8000/fs,   2*9000/fs,  2*10000/fs  }; 
float x; //input from mic
volatile static float XBuff[N] ; //circular buffer utilized by the filter function, static used so all values automatically initialized to zero
volatile int xWrite = 0; //write index for the circular buffer, easier to keep track of than pointer to element in XBuff;
int yWrite = 0; //write index for the output values of the filters

// do filter output all with one struct
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


void setup() {
  pinMode(adc_pin, INPUT);
  adc.setAveraging(1);
  adc.setResolution(12);
  adc.setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED);
  adc.setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);
  adc.adc0->analogRead(adc_pin); // performs various ADC setup stuff
  adc.enableInterrupts(ADC_0);

  if(adc.adc0->fail_flag) {
      Serial.print("ADC error: ");
      Serial.println(adc.adc0->fail_flag, HEX);
  }

  adc.adc0->stopPDB();
  const uint32_t pdb_trigger_frequency = 50000;
  adc.adc0->startPDB(pdb_trigger_frequency);

}

void loop() {
  //example of how filter will be used 
  //assuming mic input for current time is held in *x
  //would be assigned on sample ready interrupt normally
  //XBuff[xWrite] = x;
  
  
}

float filter(volatile float XBuff[], const float P[], float fc, int writeInd){
  float out = 0; //output for current time
  for(int convInd=0; convInd<N; convInd++){ //perform the convolution
    int revInd = (N-convInd)%N;
    int readInd = (convInd+writeInd)%N;
    out += 2 * P[revInd] * XBuff[readInd] * cosf(PI * fc * float(revInd)); //weird indexing on P is because of how circular buffer is constructed s.t. [... x[-1] x[0] x[-99] ...]
  }
  return out;
}

void adc0_isr() {
  XBuff[xWrite] = adc.adc0->readSingle();
  
  
  YOUT.y_0[yWrite] = filter(XBuff, P, FC[0], xWrite);//FC[index] is the frequency that you want to filter for, the fractions correspond to [1000 Hz 2000 Hz....10000 Hz]
  //do for each output channel
  YOUT.y_1[yWrite] = filter(XBuff, P, FC[1], xWrite);
  YOUT.y_2[yWrite] = filter(XBuff, P, FC[2], xWrite);
  YOUT.y_3[yWrite] = filter(XBuff, P, FC[3], xWrite);
  YOUT.y_4[yWrite] = filter(XBuff, P, FC[4], xWrite);
  YOUT.y_5[yWrite] = filter(XBuff, P, FC[5], xWrite);
  YOUT.y_6[yWrite] = filter(XBuff, P, FC[6], xWrite);
  YOUT.y_7[yWrite] = filter(XBuff, P, FC[7], xWrite);
  YOUT.y_8[yWrite] = filter(XBuff, P, FC[8], xWrite);
  YOUT.y_9[yWrite] = filter(XBuff, P, FC[9], xWrite);

  
  yWrite++;
  yWrite %= L;

  xWrite++;
  xWrite %= N;


  
}


