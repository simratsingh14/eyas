#include "digitalWriteFast.h"

// left motor pins
#define InAL            10                      // INA motor pin
#define InBL            11                      // INB motor pin
#define PWML            6                       // PWM motor pin
#define encodPinAL      2                       // encoder A pin
#define encodPinBL      3                       // encoder B pin

//right motor pins
#define InAR            12                      // INA motor pin
#define InBR            13                      // INB motor pin
#define PWMR            5                       // PWM motor pin
#define encodPinAR      18                       // encoder A pin
#define encodPinBR      19                       // encoder B pin

unsigned long start_Time = millis();
volatile int countLeft = 0;
volatile int countRight = 0;
volatile int left_prev_count = 0;
volatile int right_prev_count = 0;
volatile int left_RPM = 0;
volatile int right_RPM = 0;
volatile float left_encoder_count =0;
volatile float right_encoder_count =0;
int Leftoffset = 0;
int Rightoffset = 0;
float konstant = 11.11;
float velocity = 0;
float eeta = 0.3;

//Timer<1,millis> timer;

void setup() 
{
  TCCR1B = 0x00;     // Stop Timer
  TCNT1  = 0xFB80;  // 0.02s
  OCR1A  = 0x0000;  // Output Compare Register (OCR) - Not used
  OCR1B  = 0x0000;  // Output Compare Register (OCR) - Not used
  OCR1C  = 0x0000;  // Output Compare Register (OCR) - Not used
  ICR1   = 0x0000;  // Input Capture Register (ICR)  - Not used
  TCCR1A = 0x00;
  TCCR1C = 0x00;
  TIMSK1 = 0x01;    // Enable Timer Overflow Interrupt

  // put your setup code here, to run once:
  pinMode(InAL, OUTPUT);
  pinMode(InBL, OUTPUT);
  pinMode(PWML, OUTPUT);
  pinMode(encodPinAL, INPUT_PULLUP);
  pinMode(encodPinBL, INPUT);
  digitalWrite(encodPinAL, HIGH);                      // turn on pullup resistor
  attachInterrupt(digitalPinToInterrupt(2), ISRL, CHANGE);

  pinMode(InAR, OUTPUT);
  pinMode(InBR, OUTPUT);
  pinMode(PWMR, OUTPUT);
  pinMode(encodPinAR, INPUT_PULLUP);
  pinMode(encodPinBR, INPUT);
  digitalWrite(encodPinAR, HIGH);                      // turn on pullup resistor
  attachInterrupt(digitalPinToInterrupt(18), ISRR, CHANGE);
  Serial.begin(9600);
  
}

ISR(TIMER1_OVF_vect)
{
  TCNT1 = 0xFB80;
  
  // Make a local copy of the global encoder count
  volatile float left_current_count = left_encoder_count;
  volatile float right_current_count = right_encoder_count;
  
  //     (Change in encoder count) * (60 sec/1 min)
  // RPM = __________________________________________
  //     (Change in time --> 20ms) * (PPR --> 840)
  left_RPM = (float)(((left_current_count - left_prev_count) * 60)/(0.02*270));
  right_RPM = (float)(((right_current_count - right_prev_count) * 60)/(0.02*270));
  
  // Store current encoder count for next iteration
  left_prev_count = left_current_count;
  right_prev_count = right_current_count;
}


void ISRL()
{
  int state = digitalReadFast(encodPinAL);
  if (digitalReadFast(encodPinBL))
  {
    state ? countLeft-- : countLeft++;
    
  }
  else
  {
    state ? countLeft++ : countLeft--;
  }
 }

void ISRR()
{
  int state = digitalReadFast(encodPinAR);
  if (digitalReadFast(encodPinBR))
    state ? countRight++ : countRight--;
  else
    state ? countRight-- : countRight++;}

void moveForward() {
  //analogWrite(PWML, 255);
  //analogWrite(PWMR, 255);
  digitalWrite(InAL, HIGH);
  digitalWrite(InBL, LOW);
  digitalWrite(InAR, HIGH);
  digitalWrite(InBR, LOW);
}

void  moveBackward() {
  //analogWrite(PWML, 255);
  //analogWrite(PWMR, 255);
  digitalWrite(InAL, LOW);
  digitalWrite(InBL, HIGH);
  digitalWrite(InAR, LOW);
  digitalWrite(InBR, HIGH);
}

void stopMotor(){
  digitalWrite(InAL, LOW);
  digitalWrite(InBL, LOW);
  digitalWrite(InAR, LOW);
  digitalWrite(InBL, LOW); 
}

void moveMotor(int Left, int Right)
{
  if(Left>0 && Right>0){
    analogWrite(PWML,Left);
    analogWrite(PWMR,Right);
    moveForward();
  }
  else if(Left<0 && Right<0){
  analogWrite(PWML,-1*Left);
  analogWrite(PWMR,-1*Right);
  moveBackward();
  }

   else if(Left==0 || Right==0)
   {
    stopMotor();
  
  }

}

void botVelocity(){
  velocity = (left_RPM + right_RPM)/2.0;  
}

/*int getleft_RPM()
  {
      left_RPM = countLeft*konstant;
      Serial.print("Left RPM:");
      Serial.println(left_RPM);
      countLeft = 0;
    return left_RPM;
  }
   
int getRight_RPM()
  {
    right_RPM = countRight*konstant;
    Serial.print("Right RPM:");
    Serial.println(right_RPM);
    countRight = 0;
        
  }
*/
void loop() 
{
  moveMotor(100 + Leftoffset,100 + Rightoffset);
  /*if(millis()%20==0)
  {
    getRight_RPM();
    getleft_RPM();
  }*/  
  //Serial.print("difference");
  //Serial.println(countLeft-countRight);
  Serial.print("Left RPM:");
  Serial.println(left_RPM);

  Serial.print("Right RPM:");
  Serial.println(right_RPM);
  /*Serial.print("LC");
  Serial.println(countLeft);
  Serial.print("RC");
  Serial.println(countRight);*/
  
  
}  
