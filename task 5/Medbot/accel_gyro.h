/*
  ▪ * Team Id: eYRC#804
  ▪ * Author List:
  ▪ * Filename: accel_gyro.h
  ▪ * Theme: Biped Patrol
  ▪ * Functions: mpu_init(), test(), set_offsets(), read_accel(), read_gyro(), low_pass_filter(float a[0], a[1], a[2]), high_pass_filter(g[0], g[1], g[2]), complimentary_filter_roll()
  ▪ */

/*Global Variables*/
int16_t ax, ay, az, gx, gy, gz, gnx, gny, gnz;
float a[3] = {0, 0, 0}, g[3] = {0, 0, 0};

const float pi = 3.14159265359, f_cut = 5, dT = 0.003, comp_alpha = 0.01;
float Tau = 1 / (2 * pi*f_cut);
float alpha = Tau / (Tau + dT);
float roll_deg = 0;
float roll = 0, omega = 0;

const float accel_sf = 16384, gyro_sf = 131;
float lpx = 0, lpy = 0, lpz = 0, hpx = 0, hpy = 0, hpz = 0;
int m = 1, n = 1;
/******************/
MPU6050 mpu;

void test()
{
  Serial2.println("Testing device connections...");
  Serial2.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}

void set_offsets()
{
  mpu.setXAccelOffset(-5699);
  mpu.setYAccelOffset(-645);
  mpu.setZAccelOffset(1237);

  mpu.setXGyroOffset(-104);
  mpu.setYGyroOffset(15);
  mpu.setZGyroOffset(-10);
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

void read_accel()
{
  mpu.getAcceleration(&ax, &ay, &az);
  a[0] = (ax / accel_sf);
  a[1] = (ay / accel_sf);
  a[2] = (az / accel_sf);
  low_pass_filter(a[0], a[1], a[2]);
}

void read_gyro()
{
  mpu.getRotation(&gx, &gy, &gz);
  g[0] = (gx / gyro_sf);
  g[1] = (gy / gyro_sf);
  g[2] = (gz / gyro_sf);
  high_pass_filter(g[0], g[1], g[2]);
}

void complimentary_filter_roll()
{
  roll_deg = (1 - comp_alpha) * (roll_deg + g[1] * dT) + (comp_alpha) * (atan(a[0] / abs(a[2]))) * (180 / 3.14);
  roll = roll_deg * (3.14 / 180);
  omega = g[1] * (pi / 180);
}

void mpu_init()
{
  mpu.initialize();
  test();
  mpu.setFullScaleAccelRange(0);
  mpu.setFullScaleGyroRange(0);
  set_offsets();
}
