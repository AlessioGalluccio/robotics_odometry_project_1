 # Robotics Odometry Project

## Structure of the document:
- description of the files
- parameters
- topics
- structure of the tf tree
- structure of the custom message
- how to start and use nodes, dynamic reconfiguration and services
- important info


## DESCRIPTION OF THE FILES

estimation_notebook/Apparent_baseline_and_Gear_ratio_estimation.ipynb -> python notebook that I used to estimate the apparent baseline and the gear ratio from bag1 (using bagpy)
							from both odom and gt_pose values. 
							The notebook is set up to automatically download the dataset needed from a repository of mine and it explains the method used.
							See "important info" for more details

package_1 -> the ROS package created

in package_1:
									
- src/odometry.cpp -> code of the odometry node
- cfg/SetMethod.cfg -> configuration of the dynamic reconfigure parameter for the integration method
- msg/OdomAndMethod.msg -> definition of the custom message used in topic /odom_and_method
- srv/ResetPose -> definition of the request and response message of the reset_pose service. No variable are defined because this method doesn't use them.
- srv/SetPose -> definition of the request and response message of the set_pose service. No response variables are defined because it only takes input.
- launch/odometry.launch -> file that launches node odometry and a static_transform_publisher node for the tf tree. Starting parameters can be set here.
- CMakeLists.txt -> file used to compile the package
- package.xml -> file used to compile the package



## PARAMETERS

- x -> the axis x starting position of the robot (double)
- y -> the axis y starting position of the robot (double)
- o -> the starting rotation of the robot (double)
- method -> the initial selected integration method. It can be set to "euler" (value: 0) or "rk" (value: 1)
- t -> starting time of the simulation (double)

I added the parameter t because some bags could have different starting times. 

## TOPICS

- /odom_only -> topic where only nav_msgs/Odometry messages are published. Useful for rviz
- /odom_and_method -> topic where the custom message OdomAndMethod is published, as requested by the project.
- /speeds -> topic where geometry_msgs::TwistStamped messages of the speeds of the robot are published, as requested by the project

## STRUCTURE OF THE TF TREE

world -> odom -> base_link
(root)            (leaf)

I used a static transform publisher from world to odom frame. I used the initial pose of the gt_pose of bag1 to set up the transformation.
The trasform from odom to base_link is handled in odometry.cpp in the method "callback_all_messages".

## STRUCTURE OF THE CUSTOM MESSAGE

this message appears in the topic /odom_and_method:

OdomAndMethod 

|

——> nav_msgs/Odometry odom  //odometry message

|

——> std_msgs/String method //integration method used. "euler" for euler, "rk" for runge-kutta

## HOW TO START AND USE THE NODES, DYNAMIC RECONFIGURATION AND SERVICES

Before starting the nodes, remember to include the "robotics_hw1" package in your environment!

-To start the nodes, use the command:

	roslaunch project_1 odometry.launch

	//this command will launch both the odometry node (called "odometry") and the static_transform_publisher node (called "link_world_broadcaster")
	//to see the odometry in rviz, use the /odom_only topic. You can set the "world", "odom" and "base_link" frames 

-To use the dynamic reconfiguration of the integration method:

	rosrun dynamic_reconfigure dynparam set /odom_mine method euler
	rosrun dynamic_reconfigure dynparam set /odom_mine method rk
	//"euler" set to euler method, "rk" set to runge-kutta

-To use the "set odometry to (x,y,o)" service:

	rosservice call /set_pose 3.0 2.0 1.0
	//this will set the odometry at x:3.0, y:2.0, orientation:1.0

-To use the "reset odometry to (0,0,0)" service:

	rosservice call /reset_pose
	//I decided to set the orientation to 0 radians too.


## IMPORTANT INFO

- The method used for the estimation of the apparent baseline and the gear ratio can be found in the notebook Apparent_baseline_and_Gear_ratio_estimation.ipynb in estimation_notebook folder.
I used the sum of the squared errors of position x, y and orientation as loss function and I did a simple search for both parameters in an interval of reasonable values.
As dataset I used the values of bag1. I extracted them in csv files using the bagpy library.
I used this approach because it seemed more systematic and professional to me than doing random experiments, and it permitted me to do a fast and accurate search of the best parameters. 
As simple as it was as an optimization method, I was able to find clear minimums for the apparent baseline and the gear ratio. 
These minimums were almost the same using data from "odom" or from "gt_pose". Only a difference of 1-2 millimeters can be found in the value of the apparent baseline.
 
Gear ratio estimated: 40
Apparent baseline estimated: 0.988 m (Y0: 0.494 m)

- The notebook was written using google colab and it uses commands to automatically download the dataset from a repository of mine. No manual adding of data required.

- In the service to reset the odometry to (0,0), I decided to set also the orientation to 0.
