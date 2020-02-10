#include <Arduino.h>

//VCC  -  5V
//GND  -  GND
//SDA  -  20
//SCL  -  21
#include "I2Cdev.h"
#include "MPU6050.h"
#include<Wire.h>
#include<math.h>

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
#define encodPinAR      18                      // encoder A pin
#define encodPinBR      19                      // encoder B pin

// creating object of class MPU6050
// class default I2C address is 0x68
MPU6050 mpu;

int n = 1;
int m = 1;

float comp_alpha = 0.03;
float roll = 0;

//accelerometer and gyroscope readings
int16_t ax, ay, az;
int16_t gx, gy, gz, gnx, gny, gnz;
float a[3] = {0, 0, 0};
float g[3] = {0, 0, 0};

//filter constants
float pi = 3.14;
float f_cut = 5;
float dT = 0.01;
float Tau = (1 / (2 * pi*f_cut));            // Time constant
float alpha = (Tau / (Tau + dT));

//scaling factor
float accel_sf = 16384;
float gyro_sf = 131;

//variables for filter
float lpx = 0;
float lpy = 0;
float lpz = 0;
float hpx = 0;
float hpy = 0;
float hpz = 0;

volatile float left_encoder_count = 0;
volatile float right_encoder_count = 0;

float left_prev_count=0;
float right_prev_count=0;
float right_RPM = 0;
float left_RPM = 0;

void read_accel() {
  // read raw accel readings
  mpu.getAcceleration(&ax, &ay, &az);

  a[0] = (ax / accel_sf);
  a[1] = (ay / accel_sf);
  a[2] = (az / accel_sf);
  // Serial.println(a[0]);
  low_pass_filter(a[0], a[1], a[2]);

}

void read_gyro() {
  //read raw gyro raedings

  mpu.getRotation(&gx, &gy, &gz);

  g[0] = (gx / gyro_sf);
  g[1] = (gy / gyro_sf);
  g[2] = (gz / gyro_sf);

  high_pass_filter(g[0], g[1], g[2]);

}
void low_pass_filter(float Ax, float Ay, float Az)
{
  if (n == 1) {
    lpx = (1 - alpha) * Ax;
    lpy = (1 - alpha) * Ay;
    lpz = (1 - alpha) * Az;
    n++;
  }
  else
  {
    lpx = (1 - alpha) * Ax + alpha * lpx;
    lpy = (1 - alpha) * Ay + alpha * lpy;
    lpz = (1 - alpha) * Az + alpha * lpz;
  }
}
void high_pass_filter(float Gx, float Gy, float Gz)
{
  if (m == 1)
  {
    hpx = (1 - alpha) * Gx;
    hpy = (1 - alpha) * Gy;
    hpz = (1 - alpha) * Gz;
    m++;
  }
  else
  {
    hpx = (1 - alpha) * (hpx + Gx - gnx);
    hpy = (1 - alpha) * (hpy + Gy - gny);
    hpz = (1 - alpha) * (hpz + Gz - gnz);
  }
  gnx = Gx;
  gny = Gy;
  gnz = Gz;
}

void complimentary_filter_roll()
{
  roll = (1 - comp_alpha) * (roll - g[1] * dT) + (comp_alpha) * (atan(a[0] / abs(a[2]))) * (180 / 3.14);
}

// Function to test the connections
void test() {
  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}

// Function to set offsets obtained by running IMU_zero example code
void set_offsets() {

  //to set accelerometer offsets
  mpu.setXAccelOffset(-5699);
  mpu.setYAccelOffset(-645);
  mpu.setZAccelOffset(1237);

  //to set gyro offsets
  mpu.setXGyroOffset(-104);
  mpu.setYGyroOffset(15);
  mpu.setZGyroOffset(-10);
}

void timer1_init()
{
  TCCR1B = 0x00;    // Stop Timer
  TCNT1  = 0xFB80;  // 0.02s
  OCR1A  = 0x0000;  // Output Compare Register (OCR) - Not used
  OCR1B  = 0x0000;  // Output Compare Register (OCR) - Not used
  OCR1C  = 0x0000;  // Output Compare Register (OCR) - Not used
  ICR1   = 0x0000;  // Input Capture Register (ICR)  - Not used
  TCCR1A = 0x00;
  TCCR1C = 0x00;
}


//timer for calculating RPM of motors
void start_timer1()
{
  TCCR1B = 0x04;    // Prescaler 256 1-0-0
  TIMSK1 = 0x01;    // Enable Timer Overflow Interrupt
}


void left_encoder_interrupt(){
  
  int state = digitalRead(encodPinAL);
  if(digitalRead(encodPinBL)) 
  state ? left_encoder_count-- : left_encoder_count++;
  else 
  state ? left_encoder_count++ : left_encoder_count--;
  
}

void right_encoder_interrupt(){
  
  int state = digitalRead(encodPinAR);
  if(digitalRead(encodPinBR)) 
  state ? right_encoder_count++ : right_encoder_count--;
  else 
  state ? right_encoder_count-- : right_encoder_count++;
  
}



void motor_pin_config(){
  
  pinMode(InAL, OUTPUT);
  pinMode(InBL, OUTPUT);
  pinMode(InAR, OUTPUT);
  pinMode(InBR, OUTPUT);
  pinMode(PWML, OUTPUT);
  pinMode(PWMR, OUTPUT);
  
}

void encoder_pin_config(){
  
  pinMode(encodPinAL, INPUT_PULLUP);
  pinMode(encodPinBL, INPUT_PULLUP);
  pinMode(encodPinAR, INPUT_PULLUP);
  pinMode(encodPinBR, INPUT_PULLUP);

    // Attach interrupts for the encoder input pins
  attachInterrupt(digitalPinToInterrupt(encodPinAL), left_encoder_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encodPinAR), right_encoder_interrupt, CHANGE);
  
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
  left_RPM = (float)(((left_current_count - left_prev_count) * 60)/(0.02*540));
  right_RPM = (float)(((right_current_count - right_prev_count) * 60)/(0.02*540));
  
  // Store current encoder count for next iteration
  left_prev_count = left_current_count;
  right_prev_count = right_current_count;
}


void motor_init(){
  
  motor_pin_config();
  encoder_pin_config(); 
}

void move_motor(){
  
  digitalWrite(InAL, HIGH);
  digitalWrite(InBL, LOW);
  digitalWrite(InAR, HIGH);
  digitalWrite(InBR, LOW);
  analogWrite(PWML, 255);
  analogWrite(PWMR, 255);
}  
  
void setup() {
  
    Serial.begin(9600);

    timer1_init();
    //timer3_init();
    motor_init();
    //mpu.initialize();
    //start_timer3();
    start_timer1();
}

void loop() {
   move_motor();
   //Serial.println(left_encoder_count);
   //Serial.println(right_encoder_count); 
   Serial.print("Left RPM:");
   Serial.println(left_RPM);
   Serial.print("Right RPM:");
   Serial.println(right_RPM);
   //Serial.print("roll:");
   //Serial.println(roll);

  // put your main code here, to run repeatedly:

}
