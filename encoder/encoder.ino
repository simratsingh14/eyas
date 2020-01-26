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
#define PWMR            7                       // PWM motor pin
#define encodPinAR      18                       // encoder A pin
#define encodPinBR      19                       // encoder B pin

volatile int countLeft = 0;
volatile int countRight = 0;



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

  Serial.begin(9600);
    moveForward();
    delay(1000);
    
    
  
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
  analogWrite(PWML, 255);
  analogWrite(PWMR, 255);
  digitalWrite(InAL, HIGH);
  digitalWrite(InBL, LOW);
  digitalWrite(InAR, HIGH);
  digitalWrite(InBR, LOW);
}

void  moveBackward() {
  analogWrite(PWML, 255);
  analogWrite(PWMR, 255);
  digitalWrite(InAL, LOW);
  digitalWrite(InBL, HIGH);
  digitalWrite(InAR, LOW);
  digitalWrite(InBR, HIGH);
}


/*void loop() {
    moveForward();
    delay(1000);
    Serial.print("Left Count Forward: ");
    Serial.println(countLeft);
    moveBackward();
    delay(1000);
    Serial.print("Left Count Backward: ");
    Serial.println(countLeft);
  // put your main code here, to run repeatedly:
  /*Serial.print("Right Count: ");
    Serial.println(countRight);
    Serial.print("Left Count: ");
    Serial.println(countLeft);
    Serial.println();
    delay(500);
  

}*/void loop(){

}
