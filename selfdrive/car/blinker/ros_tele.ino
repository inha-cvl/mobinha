int input_left = 8;
int input_right = 9;
char arrow;

#include <ros.h>
#include <std_msgs/Int8.h>

ros::NodeHandle nh;

void Blinker_cb(const std_msgs::Int8& cmd_msg){
  arrow = cmd_msg.data;
}     

void setup()
{
    pinMode(input_left, OUTPUT);
    pinMode(input, OUTPUT);
}

void loop()
{
  if(arrow == 0)
  {
    digitalWrite(input_left, Low)
    digitalWrite(input, Low)
  }
  else if(arrow == 1)
  {
    digitalWrite(input_left, High)
    digitalWrite(input, Low)
  }
  else if(arrow == 2)
  {
    digitalWrite(input_left, Low)
    digitalWrite(input, High)
  }
}
