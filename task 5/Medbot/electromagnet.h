/*
  ▪ * Team Id: eYRC#804
  ▪ * Author List:
  ▪ * Filename: electromagnet.h
  ▪ * Theme: Biped Patrol
  ▪ * Functions: MAG_init()
  ▪
*/

#define Mag1            31
#define Mag2            8

void MAG_init()
{
  pinMode(Mag1, OUTPUT);
  pinMode(Mag2, OUTPUT);
  digitalWrite(Mag1, LOW);
  digitalWrite(Mag2, LOW);
}
