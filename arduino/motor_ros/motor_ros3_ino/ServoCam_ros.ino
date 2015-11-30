// Sweep
// by BARRAGAN <http://barraganstudio.com> 
// This example code is in the public domain.


#include <Servo.h> 
#include <ros.h>
#include <std_msgs/Char.h>
#include <MsTimer2.h>

Servo servox;  // create servo object to control a servo 
Servo servoy;                // a maximum of eight servo objects can be created 
 
int posx = 90;    // variable to store the servo position 
int posy = 90;

ros::NodeHandle nh;
//command
std_msgs::Char cmd_servox;
std_msgs::Char cmd_servoy;

inline int mkp(char s) {
   switch(s) {
      case '+': return +1;
      case '-': return -1;
      case '0': return 0;
   } 
   return 0;
}


void servo_x(const std_msgs::Char &cmd_servox)
{
  char x[3] = {'0', '+', '-'}; //   left(+) right(-)
  int stepx = mkp(cmd_servox.data);
  posx += stepx;
  if(posx > 180) posx = 178;
  if(posx < 0) posx = 0;
  servox.write(posx);
}

void servo_y(const std_msgs::Char &cmd_servoy)
{
  char y[3] = {'0', '+', '-'}; //    up(-)  down(+)
  int stepy = mkp(cmd_servoy.data);
  posy += stepy;
  if(posy > 180) posy = 178;
  if(posy < 55) posy = 55;
  servoy.write(posy); 
}

ros::Subscriber<std_msgs::Char>sub_servox("servox_chatter", servo_x);
ros::Subscriber<std_msgs::Char>sub_servoy("servoy_chatter", servo_y);

void setup() 
{ 
  servox.attach(9);  // attaches the servo on pin 9 to the servo object 
  servoy.attach(10);
  servox.write(posx);
  servoy.write(posy);

  nh.initNode();
  nh.subscribe(sub_servox);
  nh.subscribe(sub_servoy);
} 
 

void loop() 
{ 
  nh.spinOnce();
  delay(100);
}

