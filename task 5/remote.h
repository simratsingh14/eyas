  /*
  ▪ * Team Id: eYRC#804
  ▪ * Author List:
  ▪ * Filename: remote.h
  ▪ * Theme: Biped Patrol
  ▪ * Functions: read_joystick()
  ▪ */
/*Global Variables*/
int x_axis_val = 0, y_axis_val = 0;
int button_1 = 0, button_2 = 0, button_3 = 0, button_4 = 0;
bool forward_flag = false;
bool backward_flag = false;
bool right_flag = false;
bool left_flag = false;
bool slope_flag = false;


//joystick limits
const int x_upper = 600, x_lower = 400, y_upper = 600, y_lower = 400;
/*****************/

void read_joystick()
{
  int sum = 131;
  unsigned char byte_discard, checksum;
  unsigned char digital_data = 0, digital_data_1 = 0;
  unsigned char AD0[2] = {0, 0}, AD1[2] = {0, 0};
  if (Serial2.available() >= 18)
  {
    if (Serial2.read() == 0x7E)
    {
      for (int i = 0; i < 2; i++)
        byte_discard = Serial2.read();

      if (Serial2.read() != 0x83) return;

      for (int i = 0; i < 7; i++)
        sum += Serial2.read();

      digital_data_1 = Serial2.read();
      button_1 = digital_data_1;
      sum += digital_data_1;

      digital_data = Serial2.read();
      sum += digital_data;

      AD0[1] = Serial2.read();
      AD0[0] = Serial2.read();
      AD1[1] = Serial2.read();
      AD1[0] = Serial2.read();
      sum = sum + AD0[0] + AD0[1] + AD1[0] + AD1[1];

      checksum = 0xFF - (0xFF & (unsigned char)sum);
      byte_discard = Serial2.read();

      if (byte_discard != checksum) return;

      if ((digital_data & 0x08) == 0x08)
        button_2 = 1;
      else
        button_2 = 0;

      if ((digital_data & 0x10) == 0x10)
        button_3 = 1;
      else
        button_3 = 0;

      if ((digital_data & 0x04) == 0x04)
        button_4 = 1;
      else
        button_4 = 0;

      y_axis_val = AD0[0] + (AD0[1] * 256);
      x_axis_val = AD1[0] + (AD1[1] * 256);

      if (x_axis_val >= x_upper)
      {
        forward_flag  = true;
        backward_flag = false;
        right_flag    = false;
        left_flag     = false;
        slope_flag    = false;
      }
      else if (x_axis_val <= x_lower)
      {
        forward_flag  = false;
        backward_flag = true;
        right_flag    = false;
        left_flag     = false;
        slope_flag    = false;
      }
      else if (y_axis_val >= y_upper)
      {
        forward_flag  = false;
        backward_flag = false;
        right_flag    = true;
        left_flag     = false;
        slope_flag    = false;
      }
      else if (y_axis_val <= y_lower)
      {
        forward_flag  = false;
        backward_flag = false;
        right_flag    = false;
        left_flag     = true;
        slope_flag    = false;
      }
      else
      {
        forward_flag  = false;
        backward_flag = false;
        right_flag    = false;
        left_flag     = false;
        slope_flag    = false;
      }

      if (button_1)
      {
        slope_flag = true;
      }
      else
      {
        slope_flag = false;
      }
    }
  }
}
