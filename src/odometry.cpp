#include "ros/ros.h"
#include "std_msgs/String.h"
#include "robotics_hw1/MotorSpeed.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

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
} global_coordinates;

enum IntegrationMethod {
  EULER,
  RK,
};


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

  global_coordinates rk(global_coordinates old_values, 
                            self_speed speeds, 
                            ros::Duration time_interval){
    global_coordinates result;
    double rk_correction = (speeds.v_orientation * time_interval.toSec()) / 2;
    double v_global_x = speeds.v_forward * cos(old_values.o + rk_correction);
    double v_global_y = speeds.v_forward * sin(old_values.o + rk_correction);

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
    const double RADIUS_WHEEL = 0.1575;
    const double ROTATION_PER_MINUTE = 1/(RADIUS_WHEEL*2*M_PI);
    const double Y0 = 0.49;
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
      global_coordinates new_positions;
      switch(integrationMethod){
        case RK:
          new_positions = rk(current_pos,speeds,sub_fl->header.stamp - *last_time);
        break;
        case EULER:
          new_positions = euler(current_pos,speeds,sub_fl->header.stamp - *last_time);
        break;
      }
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(new_positions.o);

      //we publish the transform over tf
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = sub_fl->header.stamp;
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_link";

      odom_trans.transform.translation.x = new_positions.x;
      odom_trans.transform.translation.y = new_positions.y;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = odom_quat;

      br.sendTransform(odom_trans);

      //we publish the odometry message over ROS
      nav_msgs::Odometry odom;
      odom.header.stamp = sub_fl->header.stamp;
      odom.header.frame_id = "odom";

      odom.pose.pose.position.x = new_positions.x;
      odom.pose.pose.position.y = new_positions.y;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = odom_quat;

      odom.child_frame_id = "base_link";
      odom.twist.twist.linear.x = speeds.v_forward * cos(current_pos.o);
      odom.twist.twist.linear.y = speeds.v_forward * sin(current_pos.o);
      odom.twist.twist.angular.z = speeds.v_orientation;

      odom_pub.publish(odom);

      //update of values of the variables of the class
      current_pos.x = new_positions.x;
      current_pos.y = new_positions.y;
      current_pos.o = new_positions.o;
      *last_time = sub_fl->header.stamp;
      //code for debug
      ROS_INFO("new pos: X: %f Y: %f o: %f", current_pos.x, current_pos.y, current_pos.o);
      ROS_INFO("Callback ALL triggered %f %f %f %f", sub_fl->rpm, sub_fr->rpm, sub_rl->rpm, sub_rr->rpm);
  }

  //set pose of the robot
  //x: position x of the robot
  //y: position y of the robot
  //o: orientation of the robot (in gradient)
  void setPose(double x, double y, double o){
    current_pos.x = x;
    current_pos.y = y;
    current_pos.o = o;
  }

  pub_sub(){
    ros::NodeHandle n;
    sub_fl.subscribe(n,"motor_speed_fl", 1);
    sub_fr.subscribe(n,"motor_speed_fr", 1);
    sub_rl.subscribe(n,"motor_speed_rl", 1);
    sub_rr.subscribe(n,"motor_speed_rr", 1);

    sync_.reset(new Sync(MySyncPolicy(10), sub_fl,sub_fr, sub_rl, sub_rr));
    sync_->registerCallback(boost::bind(&pub_sub::callback_all_messages, this, _1,_2, _3, _4));

    odom_pub = n.advertise<nav_msgs::Odometry>("odom_mine",50);
    current_pos.x = 0.0;
    current_pos.y = 0.0;
    current_pos.o = 0.0;
    last_time = new ros::Time(0.0);
    integrationMethod = RK;
  }

private: 
  ros::Subscriber sub;
  ros::Publisher odom_pub; 
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
  IntegrationMethod integrationMethod;
  
};

int main(int argc, char **argv){
  ros::init(argc, argv, "odom_mine");
  pub_sub my_pub_sub;
  ros::spin();
  return 0;
}



