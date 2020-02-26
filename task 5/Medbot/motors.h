/*
  ▪ * Team Id: eYRC#804
  ▪ * Author List:
  ▪ * Filename: motors.h
  ▪ * Theme: Biped Patrol
  ▪ * Functions: motor_init(), motor_pin_config(),encoder_pin_config(), left_encoder_interrupt(), leftB_encoder_interrupt(), right_encoder_interrupt(), rightB_encoder_interrupt()
  ▪ */

#define InAL            9
#define InBL            10
#define PWML            6
#define encodPinAL      2
#define encodPinBL      3

#define InAR            11
#define InBR            12
#define PWMR            4
#define encodPinAR      18
#define encodPinBR      19

/*Global Variables*/
volatile long left_encoder_count = 0, right_encoder_count = 0;
int left = 0, right = 0;
float left_RPM = 0, right_RPM = 0;
const int mode = 4, ppr = 270;
float meanrpm = 0, circumference = 0.2041;
float left_prev_count = 0, right_prev_count = 0;
float distance = 0, velocity = 0;
/****************/
void motor_pin_config()
{
  pinMode(InAL, OUTPUT);
  pinMode(InBL, OUTPUT);
  pinMode(InAR, OUTPUT);
  pinMode(InBR, OUTPUT);
  pinMode(PWML, OUTPUT);
  pinMode(PWMR, OUTPUT);
}

void left_encoder_interrupt()
{
  int state = digitalRead(encodPinAL);
  if (digitalRead(encodPinBL))
    state ? left_encoder_count-- : left_encoder_count++;
  else
    state ? left_encoder_count++ : left_encoder_count--;
}

void right_encoder_interrupt()
{
  int state = digitalRead(encodPinAR);
  if (digitalRead(encodPinBR))
    state ? right_encoder_count++ : right_encoder_count--;
  else
    state ? right_encoder_count-- : right_encoder_count++;
}

void leftB_encoder_interrupt()
{
  int state = digitalRead(encodPinBL);
  if (digitalRead(encodPinAL))
    state ? left_encoder_count++ : left_encoder_count--;
  else
    state ? left_encoder_count-- : left_encoder_count++;
}

void rightB_encoder_interrupt()
{
  int state = digitalRead(encodPinBR);
  if (digitalRead(encodPinAR))
    state ? right_encoder_count-- : right_encoder_count++;
  else
    state ? right_encoder_count++ : right_encoder_count--;
}
void encoder_pin_config()
{
  pinMode(encodPinAL, INPUT_PULLUP);
  pinMode(encodPinBL, INPUT_PULLUP);
  pinMode(encodPinAR, INPUT_PULLUP);
  pinMode(encodPinBR, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encodPinAL), left_encoder_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encodPinAR), right_encoder_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encodPinBL), leftB_encoder_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encodPinBR), rightB_encoder_interrupt, CHANGE);

}

void motor_init()
{
  motor_pin_config();
  encoder_pin_config();
}

void rpm()
{
  volatile long left_current_count = left_encoder_count;
  volatile long right_current_count = right_encoder_count;

  left_RPM = (float)(((left_current_count - left_prev_count) * 60) / (0.005 * ppr * mode));
  right_RPM = (float)(((right_current_count - right_prev_count) * 60) / (0.005 * ppr * mode));

  left_prev_count = left_current_count;
  right_prev_count = right_current_count;

  meanrpm = (left_RPM + right_RPM) / 2.0;

  velocity = (meanrpm * circumference) / 60;
  distance += (velocity * 0.005);
}

void moveMotor(int left, int right)
{
  if (left > 0)
  {
    digitalWrite(InAL, HIGH);
    digitalWrite(InBL, LOW);
    left = map(abs(left), 0, 255, 0, 212);
  }
  else if (left < 0)
  {
    digitalWrite(InAL, LOW);
    digitalWrite(InBL, HIGH);
    left = map(abs(left), 0, 255, 0, 212);
  }
  else
  {
    digitalWrite(InAL, LOW);
    digitalWrite(InBL, LOW);
  }

  if (right > 0)
  {
    digitalWrite(InAR, HIGH);
    digitalWrite(InBR, LOW);
    right = map(abs(right), 0, 255, 12, 255);
  }
  else if (right < 0)
  {
    digitalWrite(InAR, LOW);
    digitalWrite(InBR, HIGH);
    right = map(abs(right), 0, 255, 12, 255);
  }
  else
  {
    digitalWrite(InAR, LOW);
    digitalWrite(InBR, LOW);
  }
  analogWrite(PWML, left);
  analogWrite(PWMR, right);
}
