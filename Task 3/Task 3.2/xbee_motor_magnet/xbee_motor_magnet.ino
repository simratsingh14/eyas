#include<Arduino.h>

#define MagF            51                      //electromagnet pin

#define InR1            10                      // motor pin
#define InR2            9                       // motor pin  
#define PWMR            11                      // pwm motor pin

#define InL1            5                       // motor pin
#define InL2            4                       // motor pin 
#define PWML            6                       // pwm motor pin

#define PWM_val 100

//joystick limits
const int x_upper = 550;
const int x_lower = 100;
const int y_upper = 550;
const int y_lower = 100;

// joystick variables
int x_axis_val, y_axis_val;
int button_pin;

// motor initialisation
void motor_init(){
    pinMode(MagF, OUTPUT);
    
    pinMode(InL1, OUTPUT);
    pinMode(InL2, OUTPUT);
    pinMode(PWML, OUTPUT);

    pinMode(InR1, OUTPUT);
    pinMode(InR2, OUTPUT);
    pinMode(PWMR, OUTPUT);
}

//Electromagnet initialisation
void MAG_init(){
    pinMode(MagF, OUTPUT);
    
    digitalWrite(MagF, LOW);
}

/
void MagPick(void)  {
  digitalWrite(MagF, HIGH);
}

void MagDrop(void)  {
  digitalWrite(MagF, LOW);
}

void forward(){
  analogWrite(PWML, PWM_val);
  analogWrite(PWMR, PWM_val);
  digitalWrite(InL1,HIGH);
  digitalWrite(InL2,LOW);
  digitalWrite(InR1,HIGH);
  digitalWrite(InR2,LOW);
}

void backward(){
  analogWrite(PWML, PWM_val);
  analogWrite(PWMR, PWM_val);
  digitalWrite(InL1,LOW);
  digitalWrite(InL2,HIGH);
  digitalWrite(InR1,LOW);
  digitalWrite(InR2,HIGH);
}

void right(){
  analogWrite(PWML, PWM_val);
  analogWrite(PWMR, PWM_val);
  digitalWrite(InL1,HIGH);
  digitalWrite(InL2,LOW);
  digitalWrite(InR1,LOW);
  digitalWrite(InR2,HIGH);
}

void left(){
  analogWrite(PWML, PWM_val);
  analogWrite(PWMR, PWM_val);
  digitalWrite(InL1,LOW);
  digitalWrite(InL2,HIGH);
  digitalWrite(InR1,HIGH);
  digitalWrite(InR2,LOW);
}

void stops(){
  digitalWrite(InL1,LOW);
  digitalWrite(InL2,LOW);
  digitalWrite(InR1,LOW);
  digitalWrite(InR2,LOW);
}

void setup() {
  Serial.begin(9600);
  motor_init();
  MAG_init(); 
}

void loop() {
  
// To check frame length
if (Serial.available() >= 18)
{
  // To check start byte of the API frame received
   if (Serial.read() == 0x7E)
   {
    // Read and discard 11 bytes
     for (int i = 0; i < 11; i++)
     {
      byte discardByte = Serial.read();
     }
    
    // To read switch data
    button_pin  = Serial.read();
    
    /* To read joystick data */
    // To read x axis value
    int analogMSB  = Serial.read();
    int analogLSB = Serial.read();
    
    // To read y axis value
    int analogMSB1 = Serial.read();
    int analogLSB1 = Serial.read();
    x_axis_val = analogLSB + (analogMSB*256);        
    y_axis_val = analogLSB1 + (analogMSB1*256);

    if(button_pin == 8)
      Serial.print("high");
    else
      Serial.print("low");
    
    Serial.print(",");
    Serial.print(x_axis_val);
    Serial.print(", ");
    Serial.print(y_axis_val);
    Serial.println(); 
  } 
}
  if(button_pin == 0)
     MagPick();
  else if(button_pin == 8)
     MagDrop();
     
  /* conditions for different motion of the motors */ 
  if(((x_axis_val > x_lower) && (x_axis_val < x_upper)) && (y_axis_val == 1023)){
   forward();
   Serial.println("forward");
  }
  else if((x_axis_val < x_lower) && (y_axis_val == 1023)){
   backward();
   Serial.println("backward");
  }
  else if((y_axis_val < y_lower) && (x_axis_val == 1023)){
   left();
   Serial.println("right");
  }
  else if(((y_axis_val > y_lower) && (y_axis_val < y_upper)) && (x_axis_val == 1023)){
   right();
   Serial.println("left");
   }
   stops();
}
