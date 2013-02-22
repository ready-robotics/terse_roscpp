#ifndef __TERSE_ROSCPP_PARAMS_H
#define __TERSE_ROSCPP_PARAMS_H

#include <ros/ros.h>
#include <ros/exceptions.h>

#include <sstream>

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
      if(!nh.getParam(param_name, var)) {
        std::ostringstream oss;
        oss<<"Required parameter not found!"
          <<" Namespace: "<<nh.getNamespace()
          <<" Parameter: "<<param_name
          <<" Description: "<<description;
        throw ros::InvalidParameterException(oss.str());
      }
    }

  // XmlType is one of:
  //  - XmlRpc::XmlRpcValue::TypeBool
  //  - XmlRpc::XmlRpcValue::TypeInt
  //  - XmlRpc::XmlRpcValue::TypeDouble
  //  - XmlRpc::XmlRpcValue::TypeString
  template <class T, int XmlType>
    static void require_vector_param(
        const ros::NodeHandle &nh,
        const std::string &param_name,
        std::vector<T> &vec,
        const std::string &description = "N/A")
    {
      XmlRpc::XmlRpcValue xml_array;
      require_param(nh, param_name, xml_array, description);

      // Make sure it's an array type
      if(xml_array.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        std::ostringstream oss;
        oss<<"Requested vector parameter is not an array!"
          <<" Namespace: "<<nh.getNamespace()
          <<" Parameter: "<<param_name
          <<" Description: "<<description;
        throw ros::InvalidParameterException(oss.str());
      }

      // Resize the target vector
      vec.resize(xml_array.size());
      for (int32_t i = 0; i < xml_array.size(); ++i) 
      {
        if(xml_array[i].getType() == XmlType) {
          ROS_ERROR("Cannot parse XML-RPC types other than bool, int, double, or string!");
          return;
        }
        vec[i] = static_cast<T>(xml_array[i]);
      }
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
