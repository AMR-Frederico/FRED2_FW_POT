#include "rampa.h" // header in local directory
#define LEFT 0
#define RIGHT 1

unsigned long current_time[2] = {0,0};

long last_time[2] = {0, 0};
double speed_increment[2] = {0, 0};
double delta_time[2] = {0,0};
double last_set[2] = {0,0};
double ramp_set_speed[2] = {0,0};

double rampa(double set_speed, double acc, int side)
{
   current_time[side] = millis();

   // if the set speed changed, recalculate
   //  if(last_set!=set_speed){

   delta_time[side] = (current_time[side] - last_time[side]);
   speed_increment[side] = (acc * (delta_time[side] / 1000));

   //   / }
   last_time[side] = current_time[side];
   // last_set = set_speed;

   // rampa positive
   if (ramp_set_speed[side] < set_speed)
   {
      ramp_set_speed[side] = ramp_set_speed[side] + speed_increment[side];
      if (ramp_set_speed[side] > set_speed)
      {
         ramp_set_speed[side] = set_speed;
      }
   }
   // rampa negative
   if (ramp_set_speed[side] > set_speed)
   {
      ramp_set_speed[side] = ramp_set_speed[side] - speed_increment[side];
      if (ramp_set_speed[side] < set_speed)
      {
         ramp_set_speed[side] = set_speed;
      }
   }
   //   delay(100);
   return ramp_set_speed[side];
}

void debugPrint(double set_speed, int side)
{
   Serial.print("Set Speed: ");
   Serial.print(set_speed);
   Serial.print("| Target Speed: ");
   Serial.print(ramp_set_speed[side]);
   Serial.print("| Speed Increment: ");
   Serial.print(speed_increment[side]);
   Serial.print("| Current Time: ");
   Serial.print(current_time[side]);
   Serial.print("| Delta_time: ");
   Serial.print(delta_time[side]);

   Serial.println();
}