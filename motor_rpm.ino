//#include "digitalWriteFast.h"
#include "timer.h"

// left motor pins
#define InAL            10                      // INA motor pin
#define InBL            11                      // INB motor pin
#define PWML            6                       // PWM motor pin
#define encodPinAL      2                       // encoder A pin
#define encodPinBL      3                       // encoder B pin

//right motor pins
#define InAR            12                      // INA motor pin
#define InBR            13                      // INB motor pin
#define PWMR            7                       // PWM motor pin
#define encodPinAR      18                       // encoder A pin
#define encodPinBR      19                       // encoder B pin

volatile int countLeft = 0;
volatile int countRight = 0;
volatile int left_prev_count = 0;
volatile int right_prev_count = 0;
volatile int left_RPM = 0;
volatile int right_RPM = 0;
int Leftoffset = 0;
int Rightoffset = 0;

float velocity = 0;
float eeta = 0.3;

Timer timer;
void setup() 
{
  // put your setup code here, to run once:
  pinMode(InAL, OUTPUT);
  pinMode(InBL, OUTPUT);
  pinMode(PWML, OUTPUT);
  pinMode(encodPinAL, INPUT);
  pinMode(encodPinBL, INPUT);
  digitalWrite(encodPinAL, HIGH);                      // turn on pullup resistor
  digitalWrite(encodPinBL, HIGH);
  attachInterrupt(digitalPinToInterrupt(2), ISRL, CHANGE);

  pinMode(InAR, OUTPUT);
  pinMode(InBR, OUTPUT);
  pinMode(PWMR, OUTPUT);
  pinMode(encodPinAR, INPUT_PULLUP);
  pinMode(encodPinBR, INPUT_PULLUP);
  //digitalWrite(encodPinAR, HIGH);                      // turn on pullup resistor
  //digitalWrite(encodPinBR, HIGH);
  attachInterrupt(digitalPinToInterrupt(18), ISRR, CHANGE);
  timer.start();
  timer.setInterval(20);
  timer.setCallback(isr20ms);
  timer.start();
  Serial.begin(9600);
}


void isr20ms()
{
  // Make a local copy of the global encoder count
  volatile float current_countLeft = countLeft;
  volatile float current_countRight = countRight;
  
  //     (Change in encoder count) * (60 sec/1 min)
  // RPM = __________________________________________
  //     (Change in time --> 20ms) * (PPR --> 840)
  
  left_RPM = (float)(((current_countLeft - left_prev_count) * 60)/(0.02*270));
  right_RPM = (float)(((current_countRight - right_prev_count) * 60)/(0.02*270));
  
  // Store current encoder count for next iteration
  left_prev_count = current_countLeft;
  right_prev_count = current_countRight;
  
  
  Leftoffset = eeta*(velocity - left_RPM);
  Rightoffset = eeta*(velocity - right_RPM);

  
  
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
  Serial.print("Left Count: ");
  Serial.println(countLeft);
 }

void ISRR()
{
  int state = digitalReadFast(encodPinAR);
  if (digitalReadFast(encodPinBR))
    state ? countRight++ : countRight--;
  else
    state ? countRight-- : countRight++;
}

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
  analogWrite(PWML,Left);
  analogWrite(PWMR,Right);
  
  if(Left>0 && Right>0){
    moveForward();
  }
  else if(Left<0 && Right<0){
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


void loop() {
    timer.update();
    moveMotor(255 + Leftoffset,255 + Rightoffset);
    delay(1000);
    Serial.print("Left RPM: ");
    Serial.println(left_RPM);
    Serial.print("Right RPM ");
    Serial.println(right_RPM);
  // put your main code here, to run repeatedly:
}  
