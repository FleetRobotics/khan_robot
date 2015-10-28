/**
 * K.H.A.N. RobotHW Interface
 * \author: Jason Ziglar <jpz@vt.edu>
 * \date: 10/28/2015
 */
#include "khan_control/KHANHWInterface.h"
#include <transmission_interface/transmission_parser.h>

#include <urdf/model.h>

#include <boost/function.hpp>
#include <boost/bind.hpp>

using namespace std;

namespace khan
{

bool IsVelocityJointInterface(transmission_interface::JointInfo info)
{
  const string InterfaceName = "VelocityJointInterface";
  for(vector<string>::iterator ii = info.hardware_interfaces_.begin();
    ii != info.hardware_interfaces_.end(); ++ii)
  {
    if(*ii == InterfaceName)
    {
      return true;
    }
  }

  return false;
}

JointData::JointData(const std::string& name) :
  _name(name),
  _vel_cmd(0.0),
  _vel(0.0),
  _pos(0.0),
  _effort(0.0),
  _controller(),
  _input(),
  _output(),
  _mutex()
{
}

JointData::JointData(const JointData& rhs) :
  _name(rhs._name),
  _vel_cmd(rhs._vel_cmd),
  _vel(rhs._vel),
  _pos(rhs._pos),
  _effort(rhs._pos),
  _controller(rhs._controller),
  _input(rhs._input),
  _output(rhs._output),
  _mutex()
{
}

JointData::~JointData()
{
}

void JointData::update(const sensor_msgs::JointState::ConstPtr& msg)
{
  if(msg->name.empty())
  {
    ROS_WARN_STREAM_NAMED("KHANHWInterface", _name << " had an invalid JointState message, ignoring.");
    return;
  }

  boost::mutex::scoped_lock lock(_mutex);
  size_t counter = 0;

  for(; counter < msg->name.size(); ++counter)
  {
    if(msg->name[counter] == _name)
    {
      break;
    }
  }

  if(counter == msg->name.size())
  {
    return;
  }

  _pos = msg->position[counter];
  _vel = msg->velocity[counter];
}

KHANHWInterface::KHANHWInterface(const std::string& robot_ns) :
  hardware_interface::RobotHW(),
  _js_interface(),
  _vj_interface(),
  _transmissions(),
  _joints()
{
  ros::NodeHandle nh(robot_ns);
  std::string rd_param;

  if(!nh.searchParam("robot_description", rd_param))
  {
    ROS_WARN_STREAM_NAMED("KHANHWInterface", " Cannot find URDF from parameter server. Bailing.");
    return;
  }
  string urdf_string;
  nh.getParam(rd_param, urdf_string);

  // urdf::Model model;

  // if(!model.initString(urdf_string))
  // {
  //   ROS_WARN_STREAM_NAMED("KHANHWInterface", " Cannot parse URDF string");
  //   return;
  // }

  transmission_interface::TransmissionParser::parse(urdf_string, _transmissions);
  _joints.reserve(_transmissions.size());

  for(vector<transmission_interface::TransmissionInfo>::iterator ii = _transmissions.begin();
    ii != _transmissions.end(); ++ii)
  {
    if(ii->joints_.empty())
    {
      ROS_WARN_STREAM_NAMED("KHANHWInterface", ii->name_ << " does not have any joints assigned.");
      continue;
    }

    //grab first VelocityJointInterface in the transmission
    vector<transmission_interface::JointInfo>::iterator joint = std::find_if(ii->joints_.begin(), ii->joints_.end(),
      IsVelocityJointInterface);

    if(joint == ii->joints_.end())
    {
      ROS_WARN_STREAM_NAMED("KHANHWInterface", ii->name_ <<
        " has no valid joints with a velocity interface to control");
      continue;
    }

    _joints.push_back(boost::make_shared<JointData>(joint->name_));

    JointData& j_info = *_joints.back();
    hardware_interface::JointStateHandle base_handle =
      hardware_interface::JointStateHandle(j_info._name, &j_info._pos,
        &j_info._vel, &j_info._effort);

    _js_interface.registerHandle(base_handle);

    hardware_interface::JointHandle vel_handle = hardware_interface::JointHandle(base_handle,
      &j_info._vel_cmd);
    _vj_interface.registerHandle(vel_handle);

    //Note all pub/sub is assumed to be in $robot_ns/py_controller/joint_name
    //Inside, a JointState topic named encoder should exist, which the Python node publishes
    //joint updates
    //Inside, a JointState topic named cmd should exist, which the Python node reads for
    //velocity commands
    const string control_base = robot_ns + "/py_controller/" + j_info._name;
    j_info._input = nh.subscribe(control_base + "/encoder", 10,
      &JointData::update, &j_info);
    j_info._output = nh.advertise<sensor_msgs::JointState>(control_base + "/cmd", 10);
    //init PID controller for this joint
    j_info._controller.init(control_base, true);
  }

  registerInterface(&_js_interface);
  registerInterface(&_vj_interface);
}

/**
 * Default destructor
 */
KHANHWInterface::~KHANHWInterface()
{
}

/**
 * Write commands to every joint on the robot
 */
void KHANHWInterface::write(ros::Time time, ros::Duration period)
{
  //For every controlled joint
  for(vector<boost::shared_ptr<JointData> >::iterator ii = _joints.begin(); ii != _joints.end(); ++ii)
  {
    JointData& data = **ii;
    boost::mutex::scoped_lock lock(data._mutex);
    double error = data._vel_cmd - data._vel;
    //Compute PID command
    const double new_vel = data._controller.computeCommand(error, period);
    //Package into message
    sensor_msgs::JointState msg;
    msg.header.stamp = time;
    msg.header.frame_id = data._name;
    msg.name.push_back(data._name);
    msg.velocity.push_back(new_vel);
    //Publish commanded message
    data._output.publish(msg);
  }
}

}