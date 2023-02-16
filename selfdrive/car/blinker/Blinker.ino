int ENA=2;
int IN1=3;
int IN2=4;
int ENB=7;
int IN3=5;
int IN4=6;
char arrow;
#include <ros.h>
#include <std_msgs/Int8.h>

ros::NodeHandle nh;

void Blinker_cb(const std_msgs::Int8& cmd_msg){
  arrow = cmd_msg.data;
}            

ros::Subscriber<std_msgs::Int8> sub("Blinker", Blinker_cb);

void setup()
{
  nh.initNode();
  nh.subscribe(sub);

  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  digitalWrite(ENA,HIGH);
  digitalWrite(ENB,HIGH);
}

void loop()
{
  if(arrow == 1)
  {
    for(int i = 0 ; i < 10 ; i++)
    {
      rightBlink();
      delay(6000);
      leftBlink();
    }
  }
  else if(arrow == -1)
  {
    for(int i = 0 ; i < 10 ; i++)
    {
      leftBlink();
      delay(6000);
      rightBlink();
    }
  }
}
