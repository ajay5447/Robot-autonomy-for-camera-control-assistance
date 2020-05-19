/**
 * @file /include/autoCam_pkg/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef autoCam_pkg_QNODE_HPP_
#define autoCam_pkg_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829

#include <string>
#include <QThread>
#include <QStringListModel>
#include "common.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace autoCam_pkg {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
  QNode(int argc, char** argv);
	virtual ~QNode();
	bool init();
  void run();
  void publishControl();

Q_SIGNALS:
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;
  image_transport::Subscriber image_sub;
  image_transport::Subscriber image_sub_2;
  ros::Subscriber cartesian_sub;
  ros::Publisher control_pub;
  ros::Publisher cartesian_pub;
  ros::Publisher joint_pub;
  ros::Publisher relaxedik_pub;
  ros::Subscriber joint_sub;
  void imageCallback(const sensor_msgs::ImageConstPtr &msg);
  void imageCallback_2(const sensor_msgs::ImageConstPtr &msg);
  void cartesianCallback(const gazebo_msgs::LinkStatesConstPtr &msg);
  void jointStatesCallback(const sensor_msgs::JointStateConstPtr &msg);
};


}  // namespace autoCam_pkg

#endif /* autoCam_pkg_QNODE_HPP_ */
