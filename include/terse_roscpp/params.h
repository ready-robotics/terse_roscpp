#ifndef __TERSE_ROSCPP_PARAMS_H
#define __TERSE_ROSCPP_PARAMS_H

#include <ros/ros.h>
#include <ros/exceptions.h>

#include <terse_roscpp/param.h>

#include <sstream>

// Backwards compatibility
namespace terse_roscpp {
  // require_param //
  // Function that raises a ros::InvalidParameterException if a required
  // parameter is not found. It also reports a ROS_FATAL error message
  // describing the param and the namespace in which it was trying to find it.
  template <class T>
    static void require_param(
        const ros::NodeHandle &nh,
        const std::string &param_name,
        T &var,
        const std::string &description = "N/A")
    {
      terse_roscpp::param::get(nh, param_name, var, description, true);
    }

  template <class T, int XmlType>
    static void require_vector_param(
        const ros::NodeHandle &nh,
        const std::string &param_name,
        std::vector<T> &vec,
        const std::string &description = "N/A")
    {
      terse_roscpp::param::get(nh, param_name, vec, description, true);
    }

  template<>
    void require_param<std::vector<bool> >(
        const ros::NodeHandle &nh,
        const std::string &param_name,
        std::vector<bool> &var,
        const std::string &description)
    {
      require_vector_param<bool, XmlRpc::XmlRpcValue::TypeBoolean>(nh, param_name, var, description);
    }

  template<>
    void require_param<std::vector<int> >(
        const ros::NodeHandle &nh,
        const std::string &param_name,
        std::vector<int> &var,
        const std::string &description)
    {
      require_vector_param<int, XmlRpc::XmlRpcValue::TypeInt>(nh, param_name, var, description);
    }

  template<>
    void require_param<std::vector<double> >(
        const ros::NodeHandle &nh,
        const std::string &param_name,
        std::vector<double> &var,
        const std::string &description)
    {
      require_vector_param<double, XmlRpc::XmlRpcValue::TypeDouble>(nh, param_name, var, description);
    }

  template<>
    void require_param<std::vector<std::string> >(
        const ros::NodeHandle &nh,
        const std::string &param_name,
        std::vector<std::string> &var,
        const std::string &description)
    {
      require_vector_param<std::string, XmlRpc::XmlRpcValue::TypeString>(nh, param_name, var, description);
    }

}

#endif // ifndef __TERSE_ROSCPP_PARAMS_H
