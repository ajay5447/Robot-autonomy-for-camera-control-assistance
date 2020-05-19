#ifndef COMMON_H
#define COMMON_H

#endif // COMMON_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/LinkStates.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include "tf2/LinearMath/Matrix3x3.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#endif

namespace autoCam_pkg {
/* Image streams for cameras */
extern cv_bridge::CvImagePtr my_global_cv_ptr;
extern cv_bridge::CvImagePtr my_global_cv_ptr_2;

/* Camera robot */
extern geometry_msgs::Pose hand_camera_pose;
extern std_msgs::String controlState;
extern geometry_msgs::Pose commandCardtesianPose;
extern sensor_msgs::JointState jointState;
extern sensor_msgs::JointState commandJointPose;
extern sensor_msgs::JointState relaxedik_weights;
}
