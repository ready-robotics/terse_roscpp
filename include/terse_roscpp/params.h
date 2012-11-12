#ifndef __TERSE_ROSCPP_PARAMS_H
#define __TERSE_ROSCPP_PARAMS_H

#include <ros/ros.h>
#include <ros/exceptions.h>

namespace terse_roscpp {

  // require_param //
  // Function that raises a ros::InvalidParameterException if a required
  // parameter is not found. It also reports a ROS_FATAL error message
  // describing the param and the namespace in which it was trying to find it.
  template <class T>
    static void require_param(
        const ros::NodeHandle &nh,
        const std::string &param_name,
        T &var)
    {
      if(!nh.getParam(param_name, var)) {
        ROS_FATAL_STREAM(
            "Required parameter not found! Namespace: "<<nh.getNamespace()
            <<" Parameter: "<<param_name);
        throw ros::InvalidParameterException("Parameter not found!");
      }
    }
}

#endif // ifndef __TERSE_ROSCPP_PARAMS_H
