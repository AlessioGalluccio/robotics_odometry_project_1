#include "ros/ros.h"
#include "std_msgs/String.h"
#include "robotics_hw1/MotorSpeed.h"

#include <sstream>

class pub_sub {

public:
  robotics_hw1::MotorSpeed speed_fl;
  robotics_hw1::MotorSpeed speed_fr;
  robotics_hw1::MotorSpeed speed_rl;
  robotics_hw1::MotorSpeed speed_rr;

private:
  ros::NodeHandle n; 

  ros::Subscriber sub_fl;
  ros::Subscriber sub_fr;
  ros::Subscriber sub_rl;
  ros::Subscriber sub_rr;
  ros::Publisher pub; 
  ros::Timer timer1;
  
public:
  pub_sub(){
    sub_fl = n.subscribe("/motor_speed_fl", 1, &pub_sub::callback_fl, this);
    sub_fr = n.subscribe("/motor_speed_fr", 1, &pub_sub::callback_fr, this);
    sub_rl = n.subscribe("/motor_speed_rl", 1, &pub_sub::callback_rl, this);
    sub_rr = n.subscribe("/motor_speed_rr", 1, &pub_sub::callback_rr, this);
    pub = n.advertise<robotics_hw1::MotorSpeed>("/odom_my", 1);
    timer1 = n.createTimer(ros::Duration(1), &pub_sub::callback_t, this);
  }

  void callback_fl(const robotics_hw1::MotorSpeed msg){
    speed_fl=msg;
  }

  void callback_fr(const robotics_hw1::MotorSpeed msg){
    speed_fr=msg;
  }

  void callback_rl(const robotics_hw1::MotorSpeed msg){
    speed_rl=msg;
  }

  void callback_rr(const robotics_hw1::MotorSpeed msg){
    speed_rr=msg;
  }
  

  void callback_t(const ros::TimerEvent&) {
    pub.publish(speed_fl);
    //pub.publish(message2);
    ROS_INFO("Callback 1 triggered");
  }
  
};


int main(int argc, char **argv){
  ros::init(argc, argv, "odom_mine");
  ros::NodeHandle n;
  pub_sub my_pub_sub;
  ros::spin();
  return 0;
}

typedef struct{
    float v_forward;
    float v_rotation;
} self_speed;

typedef struct{
    float x;
    float y;
    float o;
} global_coordinates;

global_coordinates euler(global_coordinates& old_values, 
                            self_speed& speeds, 
                            float& time_interval){
    float v_global_x = speeds.v_forward * cos(old_values.o);
    float v_global_y = speeds.v_forward * sin(old_values.o);

    float next_x = old_values.x + v_global_x * time_interval;
    //FINISCI E CONTROLLA 
}