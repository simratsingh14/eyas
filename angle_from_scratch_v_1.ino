//Hard_work_version_1.0
//connections for Arduino mega board
//VCC  -  5V
//GND  -  GND
//SDA  -  20
//SCL  -  21
#include "I2Cdev.h"
#include "MPU6050.h"
#include<Wire.h>
#include<math.h>

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

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  // initializing serial communication
  Serial.begin(9600);
  // initialising mpu sensor
  mpu.initialize();
  // To test the connections
  test();
  mpu.setFullScaleAccelRange(0);
  mpu.setFullScaleGyroRange(0);
  // To set the offsets
  set_offsets();

}

void loop() {

  read_accel();
  read_gyro();
  complimentary_filter_roll();
 /* Serial.print(ax);
  Serial.print("     ");
  Serial.print(ay);
  Serial.print("     ");
  Serial.print(az);
  Serial.print("     ");*/
  Serial.print(roll);
 // Serial.print("     ");
  Serial.println();

}
