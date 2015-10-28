/**
 * ros_control node for commanding K.H.A.N. via Python
 * \author: Jason Ziglar <jpz@vt.edu>
 * \date: 10/28/2015
 */
#include "khan_control/KHANHWInterface.h"

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

controller_manager::ControllerManager* CM = NULL;
khan::KHANHWInterface* HWInterface = NULL;

void updateHW(const ros::TimerEvent& event)
{
  CM->update(event.current_real, event.current_real - event.last_real);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "KHANPythonControl");
  ros::NodeHandle nhp("~");

  std::string robot_ns;
  double period;

  nhp.param("robot_namespace", robot_ns, std::string(""));
  nhp.param("period", period, 1.0 / 50.0);
  //Ensure the namespace is valid
  if(!robot_ns.empty() && robot_ns[robot_ns.length() - 1] != '/')
  {
    robot_ns = robot_ns + "/";
  }

  HWInterface = new khan::KHANHWInterface(robot_ns);
  CM = new controller_manager::ControllerManager(HWInterface);
  ros::Timer timer = nhp.createTimer(ros::Duration(period), updateHW);

  ros::spin();
}