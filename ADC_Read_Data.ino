#include <kinetis.h>

#define freq 50000

long dt = 1/freq, ts1, ts2;
uint16_t value;
float volt;

void setup() {
  
Serial.begin(38400);


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
  volt = ((float)value/65535.0)*3.3;
  ts1 = micros();
  
 Serial.print("the value is  ");
 Serial.print(value);
 Serial.print("");
 Serial.print("the voltage is  ");
 Serial.print(volt);
 Serial.println("");
  }

}
