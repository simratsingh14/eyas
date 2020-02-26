/*
  ▪ * Team Id: eYRC#804
  ▪ * Author List:
  ▪ * Filename: Medbot.ino
  ▪ * Theme: Biped Patrol
  ▪ * Functions: lqr(), move_motor()
  ▪ */
#include "I2Cdev.h"
#include "MPU6050.h"
#include<Wire.h>
#include<math.h>
#include<TimerOne.h>
#include<TimerThree.h>
#include"accel_gyro.h"
#include"remote.h"
#include"electromagnet.h"
#include"motors.h"
#include"LQR_controller.h"

long prevtime = 0;

void readSensor()
{
  sei();
  read_accel();
  read_gyro();
  complimentary_filter_roll();
}

void setup()
{
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  Serial2.begin(9600);

  mpu_init();
  motor_init();
  MAG_init();
  Timer1.initialize(3000);
  Timer1.attachInterrupt(readSensor);
  Timer3.initialize(5000);
  Timer3.attachInterrupt(rpm);
}

void loop()
{
  read_joystick();
  if ((micros() - prevtime) >= 7000)
  {
    lqr();
    prevtime = micros();
  }
}
