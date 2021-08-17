/*
  ▪ * Team Id: eYRC#804
  ▪ * Author List:
  ▪ * Filename: LQR_controller.h
  ▪ * Theme: Biped Patrol
  ▪ * Functions: lqr(), move_motor()
  ▪ */

float reqVelocity = 0, reqDistance = 0, reqRoll = 0, reqOmega = 0;
float errorV = 0, errorD = 0, errorR = 0, errorO = 0;
float SLOPE_ANGLE = (0.01745 * 15);
int dummy = 0;
float U = 0, U_new = 0;

void lqr()
{
  float k[4] = { -34.120, -30.453, -119.949, -10.274};

  if (slope_flag)
  {
    if (backward_flag)
    {
      dummy = map(x_axis_val, 0, 400, -SLOPE_ANGLE, 0);
      reqRoll -= dummy;
      reqRoll = constrain(reqRoll, -SLOPE_ANGLE, 0 );
      reqVelocity = 0.3;
    }
    else if (forward_flag)
    {
      dummy = map(x_axis_val, 600, 1023, 0 , +SLOPE_ANGLE);
      reqRoll += dummy;
      reqRoll = constrain(reqRoll, 0, SLOPE_ANGLE);
      reqVelocity = -0.3;
    }
  }
  else
  {
    reqRoll = 0;
    if (forward_flag)
    {
      k[1]+=k[0];
      k[0] = 0;
      dummy = map(x_axis_val, 0, 400, 70, 0);
      reqVelocity = (circumference * dummy) / 60;
      distance = 0;
    }
    else if (backward_flag)
    {
      k[1]+=k[0];  
      k[0] = 0;
      dummy = map(x_axis_val, 600, 1023, 0, -70 );
      reqVelocity = (circumference * dummy) / 60 ;
      distance = 0;
    }
    else if (right_flag)
    {
      k[0] = 0;
      reqVelocity = 0.0;
      distance = 0;
    }
    else if (left_flag)
    {
      k[0] = 0;
      reqVelocity = 0.00;
      distance = 0;
    }
    else
    {
      k[1]+=k[0];
      k[0] = 0;
      reqDistance = 0;
      reqVelocity = 0;
      reqRoll = 0;
      reqOmega = 0;
    }
  }

  errorD = (distance - reqDistance);
  errorV = (velocity - reqVelocity);
  errorR = (roll - reqRoll) + 0.0174 * 3 ;
  errorO = (omega - reqOmega);

  U = ( - k[0] * errorD - k[1] * errorV - k[2] * errorR - k[3] * errorO);
  U_new = constrain(U * 255 / 12 , -255, 255);

  if (right_flag)
  {
    moveMotor(U_new - 80, U_new + 80) ;
  }
  else if (left_flag)
  {
    moveMotor(U_new + 80, U_new - 80) ;
  }
  else
  {
    moveMotor(U_new, U_new);
  }
}
