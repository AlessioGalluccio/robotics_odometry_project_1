#include "ros/ros.h"
#include "std_msgs/String.h"
#include "robotics_hw1/MotorSpeed.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sstream>

using namespace message_filters;
using namespace robotics_hw1;



class pub_sub {

public:
  MotorSpeedConstPtr speed_fl;
  MotorSpeedConstPtr speed_fr;
  MotorSpeedConstPtr speed_rl;
  MotorSpeedConstPtr speed_rr;

private:
  ros::NodeHandle n; 

  ros::Publisher pub; 
  ros::Timer timer1;
  
public:
  void callback_all_messages(const MotorSpeedConstPtr& sub_fl, const MotorSpeedConstPtr& sub_fr, const MotorSpeedConstPtr& sub_rl,const MotorSpeedConstPtr& sub_rr){
    //speed_fl = sub_fl;
    //ROS_INFO("Callback all triggered: %f", speed_fl->rpm);
    ROS_INFO("Callback ALL triggered");
  }

  void callback_t(const ros::TimerEvent&) {
    pub.publish(speed_fl);
    //pub.publish(message2);
    ROS_INFO("Callback 1 triggered");
  }

  pub_sub(){
  

    message_filters::Subscriber<MotorSpeed> sub_fl(n,"/motor_speed_fl", 1);
    message_filters::Subscriber<MotorSpeed> sub_fr(n,"/motor_speed_fr", 1);
    message_filters::Subscriber<MotorSpeed> sub_rl(n,"/motor_speed_rl", 1);
    message_filters::Subscriber<MotorSpeed> sub_rr(n,"/motor_speed_rr", 1);
    //typedef message_filters::sync_policies::ApproximateTime<MotorSpeed,MotorSpeed,MotorSpeed,MotorSpeed> MySyncPolicy;
    message_filters::TimeSynchronizer<MotorSpeed,MotorSpeed,MotorSpeed,MotorSpeed>sync(sub_fl,sub_fr, sub_rl, sub_rr, 10);
    sync.registerCallback(boost::bind(&pub_sub::callback_all_messages, this, _1,_2, _3, _4));
    pub = n.advertise<MotorSpeed>("/odom_my", 1);
    timer1 = n.createTimer(ros::Duration(1), &pub_sub::callback_t, this);
    
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