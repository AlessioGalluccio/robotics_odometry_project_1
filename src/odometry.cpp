#include "ros/ros.h"
#include "std_msgs/String.h"
#include "robotics_hw1/MotorSpeed.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_broadcaster.h>

#include <sstream>
#include <math.h>

using namespace message_filters;
using namespace robotics_hw1;

typedef struct{
    double v_forward;
    double v_orientation;
} self_speed;

typedef struct{
    double x;
    double y;
    double o;
    ros::Duration time;
} global_coordinates;


class pub_sub {

public:
  MotorSpeedConstPtr speed_fl;
  MotorSpeedConstPtr speed_fr;
  MotorSpeedConstPtr speed_rl;
  MotorSpeedConstPtr speed_rr;

  global_coordinates euler(global_coordinates old_values, 
                            self_speed speeds, 
                            ros::Duration time_interval){
    global_coordinates result;
    double v_global_x = speeds.v_forward * cos(old_values.o);
    double v_global_y = speeds.v_forward * sin(old_values.o);

    result.x = old_values.x + v_global_x * time_interval.toSec();
    result.y = old_values.y + v_global_y * time_interval.toSec();
    result.o = old_values.o + speeds.v_orientation * time_interval.toSec();
    if(result.o > M_PI){
      result.o = result.o - (2*M_PI);
    }
    if(result.o <= -M_PI){
      result.o = result.o + (2*M_PI);
    }
    return result;
  }
  self_speed skidSpeed(double fl_rpm, double fr_rpm, 
                double rl_rpm, double rr_rpm){
    const int GEAR_RATIO = 40;
    const int SECONDS_IN_MINUTE = 60;
    const double ROTATION_PER_MINUTE = 1/(0.1575*2*M_PI);
    const double Y0 = 0.39;
    double vel_right = (fr_rpm + rr_rpm)/(2*ROTATION_PER_MINUTE*GEAR_RATIO*SECONDS_IN_MINUTE);
    double vel_left = -(fl_rpm + rl_rpm)/(2*ROTATION_PER_MINUTE*GEAR_RATIO*SECONDS_IN_MINUTE);
    self_speed result;
    result.v_forward = (vel_right + vel_left)/2;
    result.v_orientation = (-vel_left + vel_right)/(2*Y0);
    return result;
  }

  void callback_all_messages(const MotorSpeedConstPtr& sub_fl, const MotorSpeedConstPtr& sub_fr, const MotorSpeedConstPtr& sub_rl,const MotorSpeedConstPtr& sub_rr){
      //speed_fl = sub_fl;
      //ROS_INFO("Callback all triggered: %f", speed_fl->rpm);
      self_speed speeds = skidSpeed(sub_fl->rpm, sub_fr->rpm, sub_rl->rpm, sub_rr->rpm);
      global_coordinates new_positions = euler(current_pos,speeds,sub_fl->header.stamp - *last_time);
      current_pos.x = new_positions.x;
      current_pos.y = new_positions.y;
      current_pos.o = new_positions.o;
      *last_time = sub_fl->header.stamp;
      ROS_INFO("new pos: X: %f Y: %f o: %f", current_pos.x, current_pos.y, current_pos.o);
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

    current_pos.x = 0.0;
    current_pos.y = 0.0;
    current_pos.o = 0.0;
    last_time = new ros::Time(0.0);
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

  global_coordinates current_pos;
  ros::Time *last_time;
  
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



