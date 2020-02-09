#include "digitalWriteFast.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include<Wire.h>
#include<math.h>

// left motor pins
#define InAL            11                      // INA motor pin
#define InBL            10                      // INB motor pin
#define PWML            6                       // PWM motor pin
#define encodPinAL      2                       // encoder A pin
#define encodPinBL      3                       // encoder B pin

//right motor pins
#define InAR            13                      // INA motor pin
#define InBR            12                      // INB motor pin
#define PWMR            5                       // PWM motor pin
#define encodPinAR      18                      // encoder A pin
#define encodPinBR      19                      // encoder B pin

// creating object of class MPU6050
// class default I2C address is 0x68

MPU6050 mpu;

int n = 1, m = 1;

float comp_alpha = 0.03;
float roll = 0;

//accelerometer and gyroscope readings
int16_t ax = 0, ay = 0, az = 0;
int16_t gx = 0, gy = 0, gz = 0, gnx = 0, gny = 0, gnz = 0;
float a[3] = {0, 0, 0};
float g[3] = {0, 0, 0};

//filter constants
float pi = 3.14, f_cut = 5, dT = 0.01;
float Tau = (1 / (2 * pi*f_cut));            // Time constant
float alpha = (Tau / (Tau + dT));

//scaling factors
float accel_sf = 16384, gyro_sf = 131;

//variables for filter
float lpx = 0, lpy = 0, lpz = 0, hpx = 0, hpy = 0, hpz = 0;

long prevtime = 0;
volatile int countLeft = 0, countRight = 0;
volatile int left_RPM = 0, right_RPM = 0;

int leftOffset = 0, rightOffset = 0;
float konstant = (60.0 / (540 * 0.01));
float angular_velocity = 0;
float circumference = 0.2042;
float eeta = 0.85;

float reqVelocity = 0, reqDistance = 0, reqAngle = 0, reqOmega = 0;

float errorV = 0, errorD = 0, errorA = 0, errorO = 0;
float velocity = 0, distance = 0, angle = 0, omega = 0;
float U = 0, U_new = 0;

void read_accel()                               // read raw accelelerometer readings
{
  mpu.getAcceleration(&ax, &ay, &az);

  a[0] = (ax / accel_sf);
  a[1] = (ay / accel_sf);
  a[2] = (az / accel_sf);

  low_pass_filter(a[0], a[1], a[2]);
}

void read_gyro()                                //read raw gyroscope readings
{
  mpu.getRotation(&gx, &gy, &gz);

  g[0] = (gx / gyro_sf);
  g[1] = (gy / gyro_sf);
  g[2] = (gz / gyro_sf);

  high_pass_filter(g[0], g[1], g[2]);

}
void low_pass_filter(float Ax, float Ay, float Az)
{
  if (n == 1)
  {
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
  roll = (1 - comp_alpha) * (roll - g[1] * dT) + (comp_alpha) * (atan(a[0] / abs(a[2])));
}

void test()                                   // Function to test the connections
{
  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}

void set_offsets()                             // Function to set offsets obtained by running IMU_zero example code
{
  //to set accelerometer offsets
  mpu.setXAccelOffset(-5712);
  mpu.setYAccelOffset(-615);
  mpu.setZAccelOffset(5061);

  //to set gyroscope offsets
  mpu.setXGyroOffset(-100);
  mpu.setYGyroOffset(20);
  mpu.setZGyroOffset(-7);
}

void setup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(230400);                               // initializing serial communication
  mpu.initialize();                                   // initialising mpu sensor
  test();                                             // To test the connections
  mpu.setFullScaleAccelRange(0);                      //
  mpu.setFullScaleGyroRange(0);                       //
  set_offsets();                                      // To set the offsets

  pinMode(33, OUTPUT);
  pinMode(8, OUTPUT);
  digitalWrite(33, LOW);
  digitalWrite(8, LOW);

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
  {
    state ? countRight++ : countRight--;
  }
  else
  {
    state ? countRight-- : countRight++;
  }
}

void moveMotor(int Left, int Right)
{
  if ((Left > 0) && (Right > 0))            // Forward Motor
  {
    analogWrite(PWML, Left);
    analogWrite(PWMR, Right);
    digitalWrite(InAL, HIGH);
    digitalWrite(InBL, LOW);
    digitalWrite(InAR, HIGH);
    digitalWrite(InBR, LOW);
  }
  else if ((Left < 0) && (Right < 0))       // Backward Motor
  {
    analogWrite(PWML, -1 * Left);
    analogWrite(PWMR, -1 * Right);
    digitalWrite(InAL, LOW);
    digitalWrite(InBL, HIGH);
    digitalWrite(InAR, LOW);
    digitalWrite(InBR, HIGH);
  }
  else if ((Left == 0) || (Right == 0))         // Stop Motor
  {
    digitalWrite(InAL, LOW);
    digitalWrite(InBL, LOW);
    digitalWrite(InAR, LOW);
    digitalWrite(InBL, LOW);
  }
}

void botVelocity()
{
  angular_velocity = (left_RPM + right_RPM) / 2.0;
  velocity = (angular_velocity * circumference) / 60;
}

void botangle()
{
  angle = roll;
}

void botOmega()
{
  omega = g[1] * (3.14 / 180);
}

void botDistance()
{
  distance += (velocity * 0.01);
}

void lqr(int leftOffset, int rightOffset)
{
  float kx = -28.0133, kv = -28.0913, ka = -97.9999 , ko = -9.1279; //-28.0133  -28.0913  -97.9999   -9.1279
  //-1.6894  -22.1638  -80.6293   -7.8828
  errorD = (reqDistance - distance);
  errorV = (reqVelocity - velocity);
  errorA = (reqAngle - roll);
  errorO = (reqOmega - omega);

  U = (-kv * errorV - kx * errorD - ka * errorA - ko * errorO);
  U_new = -constrain(U, -200, 200);
  moveMotor(U_new, U_new);
  //Serial.println(U_new - leftOffset);
}

void parameter()
{
  botDistance();
  botVelocity();
  botangle();
  botOmega();
}

void loop()
{
  if ((micros() - prevtime) >= 10000)
  {
    read_accel();
    read_gyro();
    complimentary_filter_roll();
    left_RPM = countLeft * konstant;
    right_RPM = countRight * konstant;
    parameter();
    leftOffset = eeta * (angular_velocity - left_RPM);
    rightOffset = eeta * (angular_velocity - right_RPM);
    lqr(leftOffset, rightOffset);
    countLeft = 0;
    countRight = 0;
    prevtime = micros();
  }
}
