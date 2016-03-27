#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <dynamixel_msgs/JointState.h>

#include <sstream>
#include <cmath>

#include "turntable/saveImages.h"

class turntable_class{
public:
  ros::NodeHandle n;
  ros::Publisher LA_pub;
  ros::Publisher Dyn_pub;
  ros::Subscriber LA_sub;
  ros::Subscriber Dyn_sub;

  std_msgs::Int16 LA_cmd_sm;
  std_msgs::Float64 Dyn_cmd_sm;

  float LA_commands[4];
  int LA_count;
  int LA_cmd;
  int LA_actual;

  float Dyn_actual_error;

  float Dyn_min;
  float Dyn_max;
  float Dyn_cmd;
  float Dyn_n;
  float Dyn_delta;
  int image_number;

  int Dyn_new_msg;
  int LA_new_msg;

  int Dyn_dir;

  turntable_class(){
    LA_pub = n.advertise<std_msgs::Int16>("LA_command_1", 1);
    Dyn_pub = n.advertise<std_msgs::Float64>("turntable_controller/command", 1);
    LA_sub = n.subscribe("/LA_actual_1", 0, &turntable_class::la_cb, this);
    Dyn_sub = n.subscribe("/turntable_controller/state",0, &turntable_class::dyn_cb, this);
    
    Dyn_min = -135;
    Dyn_max = 135;
    Dyn_cmd= Dyn_min;
    Dyn_n = 5;
    Dyn_delta = (Dyn_max - Dyn_min)/Dyn_n;
    Dyn_dir = 1;

    image_number = 0;

    LA_commands[0] = 500;
    LA_commands[1] = 600;
    LA_commands[2] = 700;
    LA_commands[3] = 800;
    LA_count = 0;
    LA_cmd = LA_commands[0];
    LA_cmd_sm.data = LA_cmd;
    std::cout << "Turntable online, going to start position\n";
    Dyn_cmd_sm.data = Dyn_cmd/180.0*M_PI;
    std::cout<< LA_cmd <<std::endl;
    Dyn_pub.publish(Dyn_cmd_sm);
    LA_pub.publish(LA_cmd_sm);
    ros::spinOnce();
    update_window_params();

    //ros::Duration(6.0).sleep();

  }

  void la_cb(const std_msgs::Int16& msg){
    LA_actual = msg.data;
    LA_new_msg = 1;

  }
  void dyn_cb(const dynamixel_msgs::JointState& msg){
    Dyn_actual_error = msg.error;
    Dyn_new_msg++ ;
    //std::cout << "NEW DYN MSG\n";
  }
  int update_turntable(){
    Dyn_cmd = Dyn_cmd + Dyn_dir*Dyn_delta;
    int reset = 0;

    if(Dyn_cmd > Dyn_max){
      Dyn_cmd = Dyn_max;
      reset = 1;
      Dyn_dir = -1;
    }

    if(Dyn_cmd < Dyn_min){
      Dyn_cmd = Dyn_min;
      reset = 1;
      Dyn_dir = 1;
    }

    Dyn_cmd_sm.data = Dyn_cmd/180.0*M_PI;
    std::cout<< Dyn_cmd_sm.data<<std::endl;
    Dyn_pub.publish(Dyn_cmd_sm);
    std::cout << "waiting to settle, command: "<< Dyn_cmd<<std::endl;
    std::cout << "turntable has settled\n";
    return reset;
  }
  void update_kinect_angle(){
    LA_count++;
    LA_cmd = LA_commands[LA_count%4];
    LA_cmd_sm.data = LA_cmd; 
    std::cout << "LA count: "<< LA_count << ", mod 4 : "<< LA_count%4<<std::endl;
    std::cout << "LA_command: "<< LA_cmd_sm.data << std::endl;
    update_window_params();

    LA_pub.publish(LA_cmd_sm);   
  }
  void update_window_params(){
    float x_window_origin[4] = {.25, .25, .25, .23};
    float y_window_origin[4] = {0.28, 0.22, 0.15, 0.0};
    float x_window_size[4] = {.35, .35, .35 ,.35};
    float y_window_size[4] = {.35, .35, .35, .42};
    int cnt = LA_count%4;
    ros::param::set("/x_window_origin", x_window_origin[cnt]);
    ros::param::set("/y_window_origin", y_window_origin[cnt]);
    ros::param::set("/x_window_size", x_window_size[cnt]);
    ros::param::set("/y_window_size", y_window_size[cnt]);
    std::cout << "Updated window params!\n";
  }

  float get_current_tt_angle(){
    return Dyn_cmd;
  }
  float get_current_kinect_angle(){
    return 20.0 + 5.0*float(LA_count % 4);
  }

  int check_angle_error(){
    float la_error;
    la_error = std::fabs(LA_cmd - LA_actual);
    std::cout << "LA Error: " << la_error << ", Dyn Error: "<< Dyn_actual_error << std::endl;
    if(Dyn_new_msg > 10 && LA_new_msg == 1){

      if(la_error < 8){
        if(std::fabs(Dyn_actual_error) < .05){
          std::cout << "Actuators have reached setpoints, pausing for 1 sec\n";
          ros::Duration(1.0).sleep();
          return 0;
        }
      }
      Dyn_new_msg = 0;
      LA_new_msg = 0;
    }
    
    LA_pub.publish(LA_cmd_sm);
    Dyn_pub.publish(Dyn_cmd_sm);
    return 1;
  }

};

int main(int argc, char **argv){
 
  ros::init(argc, argv, "turntable");
  ros::NodeHandle n;
  ros::ServiceClient save_images_client = n.serviceClient<turntable::saveImages>("save_images");
  turntable_class tt;
  ros::Rate loop_rate(5);
  int reset_angle;
  turntable::saveImages srv;
  int is_moving;
  int image_number = 0;

  while (ros::ok())
  {
      is_moving = tt.check_angle_error();
      if( is_moving == 0){
        
        srv.request.item_number = 1; // todo
        srv.request.angle = tt.get_current_kinect_angle();
        srv.request.count = image_number;
        image_number++;

        bool res = save_images_client.call(srv);
        std::cout << "response is: "<< srv.response.status<< std::endl;

        reset_angle = tt.update_turntable();
        if(reset_angle == 1){
          tt.update_kinect_angle();
        }
        ros::Duration(1.0).sleep();
      }
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}