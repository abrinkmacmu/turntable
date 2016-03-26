#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>

#include <sstream>
#include <cmath>

#include "turntable/saveImages.h"

class turntable_class{
public:
  ros::NodeHandle n;
  ros::Publisher LA_pub;
  ros::Publisher Dyn_pub;

  std_msgs::Int16 LA_cmd_sm;
  std_msgs::Float64 Dyn_cmd_sm;

  float LA_commands[4];
  int LA_count;

  float Dyn_min;
  float Dyn_max;
  float Dyn_cmd;
  float Dyn_n;
  float Dyn_delta;

  turntable_class(){
    LA_pub = n.advertise<std_msgs::Int16>("LA_command_1", 1);
    Dyn_pub = n.advertise<std_msgs::Float64>("turntable_controller/command", 1);
    
    Dyn_min = -135;
    Dyn_max = 135;
    Dyn_cmd= Dyn_min;
    Dyn_n = 5;
    Dyn_delta = (Dyn_max - Dyn_min)/Dyn_n;

    LA_commands[0] = 150;
    LA_commands[1] = 300;
    LA_commands[2] = 600;
    LA_commands[3] = 900;
    LA_count = 0;
    LA_cmd_sm.data = LA_commands[LA_count];
    LA_pub.publish(LA_cmd_sm);
    ros::Duration(4.0).sleep();

  }
  int update_turntable(){
    Dyn_cmd += Dyn_delta;
    float additional_delay = 0;
    int reset = 0;
    if(Dyn_cmd > Dyn_max){
      Dyn_cmd = Dyn_min;
      additional_delay = 5.0;
      reset = 1;
    }

    Dyn_cmd_sm.data = Dyn_cmd/180.0*M_PI;
    std::cout<< Dyn_cmd_sm.data<<std::endl;
    Dyn_pub.publish(Dyn_cmd_sm);
    std::cout << "waiting to settle...\n";
    ros::Duration(3.0+additional_delay).sleep();
    std::cout << "turntable has settled\n";
    return reset;
  }
  void update_kinect_angle(){
    LA_count++;
    LA_cmd_sm.data = LA_commands[LA_count%4];
    std::cout << "LA count: "<< LA_count << ", mod 4 : "<< LA_count%4<<std::endl;

    LA_pub.publish(LA_cmd_sm);
    ros::Duration(4.0).sleep();   
  }

  float get_current_tt_angle(){
    return Dyn_cmd/180.0*M_PI;
  }
  float get_current_kinect_angle(){
    return 15.0*float(LA_count % 4);
  }

};

int main(int argc, char **argv){
 
  ros::init(argc, argv, "turntable");
  ros::NodeHandle n;
  ros::ServiceClient save_images_client = n.serviceClient<turntable::saveImages>("save_images");
  turntable_class tt;
  ros::Rate loop_rate(10);
  int reset_angle;
  turntable::saveImages srv;

  float current_tt_angle, current_kinect_angle, image_number;
  current_kinect_angle = tt.get_current_kinect_angle();

  while (ros::ok())
  {
    
    reset_angle = tt.update_turntable();
    current_tt_angle = tt.get_current_tt_angle();
    if(reset_angle == 1){
      tt.update_kinect_angle();
    }
      
    srv.request.item_number = 1;
    srv.request.angle = 30;
    srv.request.count = 10;
    bool res = save_images_client.call(srv);
    std::cout << "response is: "<< srv.response.status<< std::endl;
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}