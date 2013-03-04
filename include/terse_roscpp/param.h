#ifndef __TERSE_ROSCPP_PARAM_H
#define __TERSE_ROSCPP_PARAM_H

#include <ros/ros.h>
#include <ros/exceptions.h>

#include <sstream>

namespace terse_roscpp {
  namespace param {

    /** Get a string describing an XmlRpc type **/
    static inline std::string get_xml_rpc_type_name(int XmlType) {
      switch(XmlType) {
        case XmlRpc::XmlRpcValue::TypeInvalid: return std::string("Invalid");
        case XmlRpc::XmlRpcValue::TypeBoolean: return std::string("Boolean");
        case XmlRpc::XmlRpcValue::TypeInt: return std::string("Int");
        case XmlRpc::XmlRpcValue::TypeDouble: return std::string("Double");
        case XmlRpc::XmlRpcValue::TypeString: return std::string("String");
        case XmlRpc::XmlRpcValue::TypeDateTime: return std::string("DateTime");
        case XmlRpc::XmlRpcValue::TypeBase64: return std::string("Base64");
        case XmlRpc::XmlRpcValue::TypeArray: return std::string("Array");
        case XmlRpc::XmlRpcValue::TypeStruct: return std::string("Struct");
      };
      return std::string("Unknown");
    }

    template <class T>
      static bool get(
          const ros::NodeHandle &nh,
          const std::string &param_name,
          const std::string &description,
          T &var,
          const bool required)
      {
        if(!nh.getParam(param_name, var)) {
          if(required) {
            std::ostringstream oss;
            oss<<"Requested parameter not found!"
              <<" Namespace: "<<nh.getNamespace()
              <<" Parameter: "<<param_name
              <<" Description: "<<description;
            throw ros::InvalidParameterException(oss.str());
          } else {
            return false;
          }
        }

        return true;
      }

    // XmlType is one of:
    //  - XmlRpc::XmlRpcValue::TypeBoolean
    //  - XmlRpc::XmlRpcValue::TypeInt
    //  - XmlRpc::XmlRpcValue::TypeDouble
    //  - XmlRpc::XmlRpcValue::TypeString
    template <class T, int XmlType>
      static bool get(
          const ros::NodeHandle &nh,
          const std::string &param_name,
          const std::string &description,
          std::vector<T> &vec,
          const bool required)
      {
        // Get the parameter
        XmlRpc::XmlRpcValue xml_array;
        if( !get(nh, param_name, description, xml_array, required) ) {
          return false;
        }

        // Make sure it's an array type
        if(xml_array.getType() != XmlRpc::XmlRpcValue::TypeArray) {
          std::ostringstream oss;
          oss<<"Requested vector parameter is not an array!"
            <<" Namespace: "<<nh.getNamespace()
            <<" Parameter: "<<param_name;
          throw ros::InvalidParameterException(oss.str());
        }

        // Resize the target vector
        vec.resize(xml_array.size());

        // Fill the vector with stuff
        for (int i = 0; i < xml_array.size(); i++) {
          if(xml_array[i].getType() != XmlType) {
            std::ostringstream oss;
            oss<<"Requested vector parameter is the wrong type!"
              <<" Requested type: "<<get_xml_rpc_type_name(XmlType)
              <<" Actual type: "<<get_xml_rpc_type_name(xml_array[i].getType());
            throw ros::InvalidParameterException(oss.str());
          }
          vec[i] = static_cast<T>(xml_array[i]);
        }

        return true;
      }

    template<> bool get<std::vector<bool> >(
        const ros::NodeHandle &nh,
        const std::string &param_name,
        const std::string &description,
        std::vector<bool> &var,
        const bool required)
    {
      return get<bool, XmlRpc::XmlRpcValue::TypeBoolean>(nh, param_name, description, var, required);
    }

    template<> bool get<std::vector<int> >(
        const ros::NodeHandle &nh,
        const std::string &param_name,
        const std::string &description,
        std::vector<int> &var,
        const bool required)
    {
      return get<int, XmlRpc::XmlRpcValue::TypeInt>(nh, param_name, description, var, required);
    }

    template<> bool get<std::vector<double> >(
        const ros::NodeHandle &nh,
        const std::string &param_name,
        const std::string &description,
        std::vector<double> &var,
        const bool required)
    {
      return get<double, XmlRpc::XmlRpcValue::TypeDouble>(nh, param_name, description, var, required);
    }

    template<> bool get<std::vector<std::string> >(
        const ros::NodeHandle &nh,
        const std::string &param_name,
        const std::string &description,
        std::vector<std::string> &var,
        const bool required)
    {
      return get<std::string, XmlRpc::XmlRpcValue::TypeString>(nh, param_name, description, var, required);
    }
  }
}

#endif // ifndef __TERSE_ROSCPP_PARAM_H
