#include <ros.h>
#include <std_msgs/Int16.h>


ros::NodeHandle  nh;
std_msgs::Int16 actual_1_SM;
ros::Publisher actual_1_pub("LA_actual_1", &actual_1_SM);
int command_1 = 400;
void command_1_CB( const std_msgs::Int16& msg){
  command_1 = msg.data;
}
ros::Subscriber<std_msgs::Int16> command_1_sub("LA_command_1", command_1_CB);

const int DIR1 = 5;
const int PWM1 = 6;
const int ANALOG1 = A0;
void setup(){
  nh.initNode();
  nh.advertise(actual_1_pub);

  nh.subscribe(command_1_sub);
  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  digitalWrite(DIR1,0);
  digitalWrite(PWM1,0);
}

int history[3] = {0,0,0};
int sampleandFilter(int newVal){
  
  history[2] = history[1];
  history[1] = history[0];
  history[0] = newVal;
  int middle;
  
  if        ((history[0] <= history[1]) && (history[0] <= history[2])){
    middle = (history[1] <= history[2]) ?   history[1] :  history[2];
    
  }else if  ((history[1] <= history[0]) && (history[1] <= history[2])){
    middle = (history[0] <= history[2]) ?   history[0] :  history[2];
    
  }else {
    middle = (history[0] <= history[1]) ?   history[0] :  history[1];
  }
  
  return middle;
}
float moving_average_1 = 0;
void loop(){
  int read_val = analogRead(ANALOG1);
  int pos_1 = sampleandFilter(read_val);
  moving_average_1 = .9*moving_average_1 + .1*float(pos_1);
  actual_1_SM.data = moving_average_1;
  actual_1_pub.publish(&actual_1_SM);
  
  int error = command_1 - moving_average_1;
  
  if ( abs(error) > 5){
    int dir = error > 0 ? 0: 255;
    int pwm = error*2;
    if(pwm > 255){ pwm = 255;}
    digitalWrite(DIR1,dir);
    digitalWrite(PWM1,pwm);
  }else{
    digitalWrite(DIR1,0);
    digitalWrite(PWM1,0);
  }
  
  
  nh.spinOnce();
  delay(10);
  
}
