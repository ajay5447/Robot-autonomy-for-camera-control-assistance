/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/
#include <sstream>
#include "../include/qt_kinova_pkg/qnode.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "../include/qt_kinova_pkg/common.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace autoCam_pkg {

cv_bridge::CvImagePtr my_global_cv_ptr;
cv_bridge::CvImagePtr my_global_cv_ptr_2;
geometry_msgs::Pose hand_camera_pose;
std_msgs::String controlState;
geometry_msgs::Pose commandCardtesianPose;
sensor_msgs::JointState jointState;
sensor_msgs::JointState commandJointPose;
sensor_msgs::JointState relaxedik_weights;
/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv) :
	init_argc(argc),
  init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"autoCam_pkg");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle nh;
	// Add your ros communications here.
  image_transport::ImageTransport it(nh);
  //  image_sub = it.subscribe("/usb_cam/image_raw", 1, &QNode::imageCallback, this);
  image_sub = it.subscribe("/cam/camera1/image_raw1", 1, &QNode::imageCallback, this); //static camera
  image_sub_2 = it.subscribe("/cam/camera/image_raw", 1, &QNode::imageCallback_2, this); //hand camera

  cartesian_sub = nh.subscribe("/gazebo/link_states", 1, &QNode::cartesianCallback, this); // cartesian position of hand camera

  joint_sub = nh.subscribe("/joint_states", 1, &QNode::jointStatesCallback, this);

  /* Publishers */
  control_pub = nh.advertise<std_msgs::String>("/controlState", 1);
  cartesian_pub = nh.advertise<geometry_msgs::Pose>("/uiCartesianPose", 1);
  joint_pub = nh.advertise<sensor_msgs::JointState>("/uiJointPose",5);
  relaxedik_pub = nh.advertise<sensor_msgs::JointState>("/uiRelaxedIk",5);
  start();
	return true;
}

void QNode::run() {
  while ( ros::ok() ) {
    ros::spin();
  }
  Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)

}

void QNode::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    try {
      my_global_cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void QNode::imageCallback_2(const sensor_msgs::ImageConstPtr& msg) {
  try {
    try {
      my_global_cv_ptr_2 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}
void QNode::cartesianCallback(const gazebo_msgs::LinkStatesConstPtr& msg) {
  /* Note that index = 9 corresponds to 'robot::leftbracelet_link'*/
//  std::cout << msg->name[9] << "\n";
  hand_camera_pose = msg->pose[17];
//  geometry_msgs::Point Position_of_ee = msg->pose[9].position;
//  geometry_msgs::Quaternion Orientation_of_ee = msg->pose[9].orientation;

//  std::cout << Position_of_ee.x << "\n";
//theWindow->observeCartesian();

}

void QNode::publishControl() {
  // Publish the control mode: "joint", "cartesian", or "objective"
  control_pub.publish(controlState);
  // If in cartesian mode, publish cartesian pose
  if (controlState.data == "cartesian") {
    cartesian_pub.publish(commandCardtesianPose);
  }
  if (controlState.data == "joint") {
    joint_pub.publish(commandJointPose);
  }
  relaxedik_pub.publish(relaxedik_weights);
}

void QNode::jointStatesCallback(const sensor_msgs::JointStateConstPtr &msg)
{
  jointState = *msg;
//  std::cout<<jointState.position[0];
}


}  // namespace autoCam_pkg
