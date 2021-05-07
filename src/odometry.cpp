#include "ros/ros.h"
#include "std_msgs/String.h"
#include "robotics_hw1/MotorSpeed.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_broadcaster.h>

#include <sstream>

using namespace message_filters;
using namespace robotics_hw1;



class pub_sub {

public:
  MotorSpeedConstPtr speed_fl;
  MotorSpeedConstPtr speed_fr;
  MotorSpeedConstPtr speed_rl;
  MotorSpeedConstPtr speed_rr;

  void callback_all_messages(const MotorSpeedConstPtr& sub_fl, const MotorSpeedConstPtr& sub_fr, const MotorSpeedConstPtr& sub_rl,const MotorSpeedConstPtr& sub_rr){
      //speed_fl = sub_fl;
      //ROS_INFO("Callback all triggered: %f", speed_fl->rpm);
      ROS_INFO("Callback ALL triggered %f %f %f %f", sub_fl->rpm, sub_fr->rpm, sub_rl->rpm, sub_rr->rpm);
      transform.setOrigin( tf::Vector3(0, 0, 0) );
      tf::Quaternion q;
      q.setRPY(0, 0, 0);
      transform.setRotation(q);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "turtle"));
  }

  pub_sub(){
    ros::NodeHandle n;
    sub_fl.subscribe(n,"motor_speed_fl", 1);
    sub_fr.subscribe(n,"motor_speed_fr", 1);
    sub_rl.subscribe(n,"motor_speed_rl", 1);
    sub_rr.subscribe(n,"motor_speed_rr", 1);

    sync_.reset(new Sync(MySyncPolicy(10), sub_fl,sub_fr, sub_rl, sub_rr));
    sync_->registerCallback(boost::bind(&pub_sub::callback_all_messages, this, _1,_2, _3, _4));
  }

private: 
  ros::Subscriber sub;
  ros::Publisher pub; 
  ros::Timer timer1;

  message_filters::Subscriber<MotorSpeed> sub_fl;
  message_filters::Subscriber<MotorSpeed> sub_fr;
  message_filters::Subscriber<MotorSpeed> sub_rl;
  message_filters::Subscriber<MotorSpeed> sub_rr;

  typedef message_filters::sync_policies::ApproximateTime<MotorSpeed,MotorSpeed,MotorSpeed,MotorSpeed> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;

  tf::TransformBroadcaster br;
  tf::Transform transform;
  
};


void callback_all_messages(const MotorSpeedConstPtr& sub_fl, const MotorSpeedConstPtr& sub_fr, const MotorSpeedConstPtr& sub_rl,const MotorSpeedConstPtr& sub_rr){
    //speed_fl = sub_fl;
    //ROS_INFO("Callback all triggered: %f", speed_fl->rpm);
    ROS_INFO("Callback ALL triggered %f %f %f %f", sub_fl->rpm, sub_fr->rpm, sub_rl->rpm, sub_rr->rpm);
  }

int main(int argc, char **argv){
  ros::init(argc, argv, "odom_mine");
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