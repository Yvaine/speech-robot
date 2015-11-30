#include <ros.h>
#include <std_msgs/Char.h>
#include <MsTimer2.h>


#define TEST
ros::NodeHandle nh;
std_msgs::Char cmd_motor;
int E1 = 5;     //定义M1使能端
int E2 = 6;     //定义M2使能端
int M1 = 4;    //定义M1控制端
int M2 = 7;    //定义M1控制端

int ang_step = 7;
const int TIME_COUNT1 = 8;
const int TIME_COUNT2 = 4;
int flag_w = TIME_COUNT1;
int flag_s = TIME_COUNT1;
int flag_a = TIME_COUNT2;
int flag_d = TIME_COUNT2;

volatile int flag = TIME_COUNT1;
volatile int flag2 = TIME_COUNT2;
void flash()
{
  flag--;
  flag2--;
}

void keep_stop(void){                 //停止
       digitalWrite(E1,LOW);   
       digitalWrite(E2,LOW);      
}  
 
void advance(char a,char b){           //������
  analogWrite (E1,a);             //PWM������
  digitalWrite(M1,HIGH);    
  analogWrite (E2,b);    
  digitalWrite(M2,HIGH);
}  
void back_off (char a,char b) {          //������
  analogWrite (E1,a);
  digitalWrite(M1,LOW);   
  analogWrite (E2,b);    
  digitalWrite(M2,LOW);
}
void turn_R (char a,char b) {           //������
  analogWrite (E1,a);
  digitalWrite(M1,LOW);    
  analogWrite (E2,b);    
  digitalWrite(M2,HIGH);
}
void turn_L (char a,char b) {           //������
  analogWrite (E1,a);
  digitalWrite(M1,HIGH);    
  analogWrite (E2,b);    
  digitalWrite(M2,LOW);
}
void motor(const std_msgs::Char &cmd_motor)
{
  char val = cmd_motor.data;
  if(val!=-1 && val != 'q'){
          switch(val){
             case 'w'://前进
                     advance (255,255);   //PWM调速
                     break;
             case 's'://后退
                     back_off (255,255);
                     break;
             case 'a'://左转
                     turn_L (255,255); // 2
                     break;       
             case 'd'://右转
                     turn_R (255,255);
                     break;        
            }
            MsTimer2::start();
            flag = TIME_COUNT1;
            flag2 = TIME_COUNT2;     
       }
      else keep_stop(); 
}
ros::Subscriber<std_msgs::Char>sub_motor("motor_chatter", motor);

void setup(void) { 
    int i;
    for(i=6;i<=9;i++)
    pinMode(i, OUTPUT);  
    keep_stop();
    nh.initNode();
    nh.subscribe(sub_motor);
    MsTimer2::set(10, flash)  ; // 10ms period
} 

void loop(void) { 
#ifdef TEST
  advance(255,255);
#else
  if(!flag || !flag2)
  {
    keep_stop();
    flag = TIME_COUNT1;
    flag2 = TIME_COUNT2;
  }
  nh.spinOnce();
#endif:w
}
