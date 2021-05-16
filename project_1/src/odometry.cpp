#include "ros/ros.h"
#include "std_msgs/String.h"
#include "robotics_hw1/MotorSpeed.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <project_1/OdomAndMethod.h>
#include <geometry_msgs/TwistStamped.h>
#include <dynamic_reconfigure/server.h>
#include <project_1/SetMethodConfig.h>
#include "project_1/SetPose.h"
#include "project_1/ResetPose.h"
#include <sstream>
#include <math.h>

using namespace message_filters;
using namespace robotics_hw1;

//velocity of the robot from its own frame of reference
typedef struct{
    double v_forward; //forward speed of the robot in m/s
    double v_orientation; //orientation speed of the robot in rad/s
} self_speed;

//position of the robot from the world frame of reference
typedef struct{
    double x; //position on the x axis in meters
    double y; //position on the y axis in meters
    double o; //orientation in radians
} global_coordinates;

//Integration method
enum IntegrationMethod {
  EULER, //euler method
  RK, //runge-kutta method
};


class pub_sub {

public:
    //Constructor
    pub_sub(){

      ros::NodeHandle n;
      //setting subscribers 
      sub_fl.subscribe(n,"motor_speed_fl", 1);
      sub_fr.subscribe(n,"motor_speed_fr", 1);
      sub_rl.subscribe(n,"motor_speed_rl", 1);
      sub_rr.subscribe(n,"motor_speed_rr", 1);

      //setting message filter
      sync_.reset(new Sync(MySyncPolicy(10), sub_fl,sub_fr, sub_rl, sub_rr));
      sync_->registerCallback(boost::bind(&pub_sub::callback_all_messages, this, _1,_2, _3, _4));

      //setting publishers
      odom_only_pub = n.advertise<nav_msgs::Odometry>("odom_only",50);
      odom_method_pub = n.advertise<project_1::OdomAndMethod>("odom_and_method",50);
      speeds_pub = n.advertise<geometry_msgs::TwistStamped>("speeds",50);

      //loading starting pose
      if(!n.getParam("x", current_pos.x)){
        ROS_ERROR("Failed to load x coordinate");
      }
      if(!n.getParam("y", current_pos.y)){
        ROS_ERROR("Failed to load y coordinate");
      }
      if(!n.getParam("o", current_pos.o)){
        ROS_ERROR("Failed to load o orientation");
      }

      //set time to 0
      double time;
      if(!n.getParam("t", time)){
        ROS_ERROR("Failed to load starting time");
      }
      else if(time < 0.0){
        ROS_ERROR("Negative time is invalid input");
      }
      else{
        last_time = new ros::Time(time);
      }
      

      //loading integration method
      std::string param_method;
      if(!n.getParam("method", param_method)){
        ROS_ERROR("Failed to load integration method");
      }
      if(param_method == "euler") {
        integrationMethod = EULER;
      }
      else if(param_method == "rk"){
        integrationMethod = RK;
      }
      else{
        ROS_ERROR("Integration method name is invalid");
      }

      //setting dynamic reconfiguration of the integration method
      f = boost::bind(&pub_sub::setMethodDynamic, this, _1, _2);
      serverDynamicRec.setCallback(f);

      //setting "set_pose" and "reset_pose" services
      service_pose = n.advertiseService("set_pose", &pub_sub::setPoseService, this);
      service_reset = n.advertiseService("reset_pose", &pub_sub::resetPoseService, this);
    }

  private: 


  //calculate next position using euler method
  //@old_values: last saved position of the robot
  //@speeds: forward speed and orientation speed calulated
  //@time_interval: difference from time of the next position and the last calculated one
  //returns: next position in global_coordinates
  global_coordinates euler_method(global_coordinates old_values, 
                            self_speed speeds, 
                            ros::Duration time_interval){
    global_coordinates result;
    double v_global_x = speeds.v_forward * cos(old_values.o);
    double v_global_y = speeds.v_forward * sin(old_values.o);

    result.x = old_values.x + v_global_x * time_interval.toSec();
    result.y = old_values.y + v_global_y * time_interval.toSec();
    result.o = old_values.o + speeds.v_orientation * time_interval.toSec();
    //normalise radians
    if(result.o > M_PI){
      result.o = result.o - (2*M_PI);
    }
    if(result.o <= -M_PI){
      result.o = result.o + (2*M_PI);
    }
    return result;
  }

  //calculate next position using runge-kutta method
  //@old_values: last saved position of the robot
  //@speeds: forward speed and orientation speed calulated
  //@time_interval: difference from time of the next position and the last calculated one
  //returns: next position in global_coordinates
  global_coordinates rk_method(global_coordinates old_values, 
                        self_speed speeds, 
                        ros::Duration time_interval){
    global_coordinates result;
    //calculate runge-kutta correction
    double rk_correction = (speeds.v_orientation * time_interval.toSec()) / 2;
    double v_global_x = speeds.v_forward * cos(old_values.o + rk_correction);
    double v_global_y = speeds.v_forward * sin(old_values.o + rk_correction);

    result.x = old_values.x + v_global_x * time_interval.toSec();
    result.y = old_values.y + v_global_y * time_interval.toSec();
    result.o = old_values.o + speeds.v_orientation * time_interval.toSec();
    //normalise radians
    if(result.o > M_PI){
      result.o = result.o - (2*M_PI);
    }
    if(result.o <= -M_PI){
      result.o = result.o + (2*M_PI);
    }
    return result;
  }

  //calculate forward speed and orientation speed of a skid steer robot
  //@fl_rpm: speed in rotations per minute of the motor of the front left wheel
  //@fr_rpm: speed in rotations per minute of the motor of the front right wheel
  //@rl_rpm: speed in rotations per minute of the motor of the rear left wheel
  //@rr_rpm: speed in rotations per minute of the motor of the rear right wheel
  //returns: forward speed and orientation in self_speed format
  self_speed skidSpeed(double fl_rpm, double fr_rpm, 
                        double rl_rpm, double rr_rpm){
    const int GEAR_RATIO = 40; //gear ratio of the motor
    const int SECONDS_IN_MINUTE = 60;
    const double RADIUS_WHEEL = 0.1575; //radius of a wheel of a robot
    const double ROTATION_PER_METER = 1/(RADIUS_WHEEL*2*M_PI); //rotations needed to move forward by a meter
    const double Y0 = 0.494; //estimated Y0
    double vel_right = (fr_rpm + rr_rpm)/(2*ROTATION_PER_METER*GEAR_RATIO*SECONDS_IN_MINUTE);
    //speeds of left side are reversed because motor is mounted in the "same way" of the right side
    double vel_left = -(fl_rpm + rl_rpm)/(2*ROTATION_PER_METER*GEAR_RATIO*SECONDS_IN_MINUTE);
    self_speed result;
    result.v_forward = (vel_right + vel_left)/2;
    result.v_orientation = (-vel_left + vel_right)/(2*Y0);
    return result;
  }

  //handle the speeds messages, update the position of the robot and publish it in the topics
  //@sub_fl: subscriber to the topic of the front left wheel speed 
  //@sub_fr: subscriber to the topic of the front right wheel speed
  //@sub_rl: subscriber to the topic of the rear left wheel speed
  //@sub_rr: subscriber to the topic of the rear right wheel speed
  void callback_all_messages(const MotorSpeedConstPtr& sub_fl, const MotorSpeedConstPtr& sub_fr,
                             const MotorSpeedConstPtr& sub_rl, const MotorSpeedConstPtr& sub_rr){
      
      global_coordinates new_positions;
      project_1::OdomAndMethod msg_odom_method;
      geometry_msgs::TwistStamped msg_speeds;

      //calulate forward and orientation speed of the robot
      self_speed speeds = skidSpeed(sub_fl->rpm, sub_fr->rpm, sub_rl->rpm, sub_rr->rpm);

      //calulate new position of the robot, using the selected integration method
      switch(integrationMethod){
        case RK:
          new_positions = rk_method(current_pos,speeds,sub_fl->header.stamp - *last_time);
          msg_odom_method.method.data = "rk";
          //ROS_INFO("method: RK");
        break;
        case EULER:
          new_positions = euler_method(current_pos,speeds,sub_fl->header.stamp - *last_time);
          msg_odom_method.method.data = "euler";
          //ROS_INFO("method: EULER");
        break;
      }

      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(new_positions.o);

      //publish the transform over tf
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = sub_fl->header.stamp;
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_link";

      odom_trans.transform.translation.x = new_positions.x;
      odom_trans.transform.translation.y = new_positions.y;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = odom_quat;

      br.sendTransform(odom_trans);

      //publish the odometry message
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

      msg_odom_method.odom = odom;
      odom_only_pub.publish(odom);
      odom_method_pub.publish(msg_odom_method);

      //publish also the speeds in /speeds
      msg_speeds.header.frame_id = "odom";
      msg_speeds.header.stamp = sub_fl->header.stamp;
      msg_speeds.twist.linear.x = speeds.v_forward * cos(current_pos.o);
      msg_speeds.twist.linear.y = speeds.v_forward * sin(current_pos.o);
      msg_speeds.twist.angular.z = speeds.v_orientation;
      speeds_pub.publish(msg_speeds);

      //update of values of the variables of the class
      current_pos.x = new_positions.x;
      current_pos.y = new_positions.y;
      current_pos.o = new_positions.o;
      *last_time = sub_fl->header.stamp;
      //code for debug
      //ROS_INFO("new pos: X: %f Y: %f o: %f", current_pos.x, current_pos.y, current_pos.o);
      //ROS_INFO("Callback ALL triggered %f %f %f %f", sub_fl->rpm, sub_fr->rpm, sub_rl->rpm, sub_rr->rpm);
      
  }

  //set pose of the robot
  //@x: position x of the robot
  //@y: position y of the robot
  //@o: orientation of the robot (in gradient)
  void setPose(double x, double y, double o){
    current_pos.x = x;
    current_pos.y = y;
    //normalise radiants if needed
    if(o > M_PI){
      current_pos.o = current_pos.o - (2*M_PI);
    }
    else if(o <= -M_PI){
      current_pos.o = current_pos.o + (2*M_PI);
    }
    else{
      current_pos.o = o;
    }
  }

  //offer dynamic reconfiguration of the integration method used
  //@config: integration method to use: 0 euler, 1 runge-kutta
  //@level: level of the dynamic configuration. 
  void setMethodDynamic(project_1::SetMethodConfig &config, uint32_t level){
      switch(config.method){
        case 0: integrationMethod = EULER;
        break;
        case 1: integrationMethod = RK;
        break;
      }
  }

  //offer service to set a new global pose of the robot
  //@req: request received in the format x, y, o (axis x, axis y, orientation)
  //@res: response parameter required for the implementation of the callback, but not used
  bool setPoseService(project_1::SetPose::Request &req, 
                      project_1::SetPose::Response &res){
    setPose(req.x,req.y,req.o);
    return true;
  }

  //set pose of the robot to (0,0,0)
  //@req: request parameter required for the implementation of the callback, but not used
  //@res: response parameter required for the implementation of the callback, but not used
  bool resetPoseService(project_1::ResetPose::Request &req, 
                      project_1::ResetPose::Response &res){
    setPose(0.0,0.0,0.0);
    return true;
  }

  //Publishers
  ros::Publisher odom_only_pub;
  ros::Publisher odom_method_pub; 
  ros::Publisher speeds_pub;

  //Subscribers
  message_filters::Subscriber<MotorSpeed> sub_fl;
  message_filters::Subscriber<MotorSpeed> sub_fr;
  message_filters::Subscriber<MotorSpeed> sub_rl;
  message_filters::Subscriber<MotorSpeed> sub_rr;

  //Policy
  typedef message_filters::sync_policies::ApproximateTime<MotorSpeed,MotorSpeed,MotorSpeed,MotorSpeed> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;

  //tf broadcaster
  tf::TransformBroadcaster br;

  //saved state of the robot
  global_coordinates current_pos;
  ros::Time *last_time;
  IntegrationMethod integrationMethod;

  //Dynamic reconfiguration
  dynamic_reconfigure::Server<project_1::SetMethodConfig> serverDynamicRec;
  dynamic_reconfigure::Server<project_1::SetMethodConfig>::CallbackType f;
  
  //Services
  ros::ServiceServer service_pose;
  ros::ServiceServer service_reset;
};

int main(int argc, char **argv){
  ros::init(argc, argv, "odometry");
  pub_sub my_pub_sub;
  ros::spin();
  return 0;
}



