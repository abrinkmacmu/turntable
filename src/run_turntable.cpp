#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>

#include <sstream>
#include <cmath>

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher LA_pub = n.advertise<std_msgs::Int16>("LA_command_1", 1);
  ros::Publisher Dyn_pub = n.advertise<std_msgs::Float64>("turntable_controller/command", 1);

  ros::Rate loop_rate(10);
  
  std_msgs::Int16 LA_cmd_sm;
  std_msgs::Float64 Dyn_cmd_sm;
  float LA_min = 150; 
  float LA_max = 850;
  float Dyn_min = -135;
  float Dyn_max = 135;

  float LA_cmd = LA_min;
  float Dyn_cmd= Dyn_min;

  float LA_n = 3;
  float Dyn_n = 20;

  float LA_delta = (LA_max - LA_min)/ LA_n;
  float Dyn_delta = (Dyn_max - Dyn_min)/Dyn_n;
  while (ros::ok())
  {
    Dyn_cmd += Dyn_delta;
    Dyn_cmd_sm.data = Dyn_cmd/180.0*M_PI;
    std::cout<< Dyn_cmd_sm.data<<std::endl;
    Dyn_pub.publish(Dyn_cmd_sm);
    ros::Duration(3.0).sleep();

    if(Dyn_cmd > Dyn_max){
      Dyn_cmd = Dyn_min;

      LA_cmd += LA_delta;
      LA_cmd_sm.data = LA_cmd;

      LA_pub.publish(LA_cmd_sm);
      ros::Duration(2.0).sleep();
      if(LA_cmd > LA_max){
        LA_cmd = LA_min;
      }
    }
    std::cout << "Dyn:"<< Dyn_cmd<<"  LA: "<<LA_cmd<<std::endl;
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}