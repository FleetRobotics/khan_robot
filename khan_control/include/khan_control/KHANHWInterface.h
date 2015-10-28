/**
 * Class Implementing the K.H.A.N. RobotHW Interface.
 * \author: Jason Ziglar <jpz@vt.edu>
 * \date: 10/28/2015
 */
#ifndef _KHAN_HW_INTERFACE_H_
#define _KHAN_HW_INTERFACE_H_

#include <sensor_msgs/JointState.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <transmission_interface/transmission_info.h>
#include <control_toolbox/pid.h>
#include <ros/ros.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <string>

namespace khan
{

/**
 * Structure storing relevant JointData. Class for copy constructors
 */
class JointData
{
public:
  JointData(const std::string& name);
  JointData(const JointData& rhs);
  ~JointData();

  void update(const sensor_msgs::JointState::ConstPtr& msg);

  std::string _name;
  double _vel_cmd;
  double _vel;
  double _pos;
  double _effort;
  control_toolbox::Pid _controller;
  ros::Subscriber _input;
  ros::Publisher _output;
  boost::mutex _mutex;
};

/**
 * HWInterface for connecting to K.H.A.N. over a separate Python interface.
 * This is not the recommended way to do this, but is useful for educational purposes.
 */
class KHANHWInterface : public hardware_interface::RobotHW
{
public:
  KHANHWInterface(const std::string& robot_ns = "/");
  virtual ~KHANHWInterface();

  //void read(ros::Time time, ros::Duration period);
  void write(ros::Time time, ros::Duration period);
private:
  hardware_interface::JointStateInterface _js_interface;
  hardware_interface::VelocityJointInterface _vj_interface;
  //! List of joints that have interfaces defined
  std::vector<transmission_interface::TransmissionInfo> _transmissions;
  std::vector<boost::shared_ptr<JointData> > _joints;
};

} //end namespace khan

#endif