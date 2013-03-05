#ifndef __TERSE_ROSCPP_PARAM_H
#define __TERSE_ROSCPP_PARAM_H

#include <ros/ros.h>
#include <ros/exceptions.h>

#include <sstream>

namespace terse_roscpp {
  namespace param {
    class TerseXmlRpcValue : public XmlRpc::XmlRpcValue {
    public:
      TerseXmlRpcValue(XmlRpc::XmlRpcValue &val) :
        XmlRpc::XmlRpcValue(val)
      { }

      operator std::map<std::string, XmlRpc::XmlRpcValue>&()
      {
        this->assertStruct();
        return *_value.asStruct;
      }
    };

    /** Get a string describing an XmlRpc type **/
    static inline std::string get_xml_rpc_type_name(int XmlType)
    {
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
          T &var,
          const std::string &description,
          const bool required = false)
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
          std::vector<T> &vec,
          const std::string &description,
          const bool required = false)
      {
        // Get the parameter
        XmlRpc::XmlRpcValue xml_array;
        if( !get(nh, param_name, xml_array, description, required) ) {
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
        std::vector<bool> &var,
        const std::string &description,
        const bool required)
    {
      return get<bool, XmlRpc::XmlRpcValue::TypeBoolean>(nh, param_name, var, description, required);
    }

    template<> bool get<std::vector<int> >(
        const ros::NodeHandle &nh,
        const std::string &param_name,
        std::vector<int> &var,
        const std::string &description,
        const bool required)
    {
      return get<int, XmlRpc::XmlRpcValue::TypeInt>(nh, param_name, var, description, required);
    }

    template<> bool get<std::vector<double> >(
        const ros::NodeHandle &nh,
        const std::string &param_name,
        std::vector<double> &var,
        const std::string &description,
        const bool required)
    {
      return get<double, XmlRpc::XmlRpcValue::TypeDouble>(nh, param_name, var, description, required);
    }

    template<> bool get<std::vector<std::string> >(
        const ros::NodeHandle &nh,
        const std::string &param_name,
        std::vector<std::string> &var,
        const std::string &description,
        const bool required)
    {
      return get<std::string, XmlRpc::XmlRpcValue::TypeString>(nh, param_name, var, description, required);
    }
    
    /////

    template <class T>
      static bool get_element(
          const ros::NodeHandle &nh,
          const std::string &param_name,
          const size_t index,
          T &var,
          const std::string &description,
          const bool required = false)
      {
        // Get the parameter
        std::vector<T> array;
        if( !get(nh, param_name, array, description, required) ) {
          return false;
        }

        // Resize the target vector
        if( index >= array.size()) {
          std::ostringstream oss;
          oss<<"Requested vector element parameter is out of range!"
            <<" Namespace: "<<nh.getNamespace()
            <<" Parameter: "<<param_name
            <<" Index: "<<index
            <<" Size: "<<array.size();
          throw ros::InvalidParameterException(oss.str());
        }

        var = array[index];

        return true;
      }
  
    static bool size(
        const ros::NodeHandle &nh,
        const std::string &param_name,
        size_t &size,
        const std::string &description,
        const bool required = false)
    {
      // Get the parameter
      XmlRpc::XmlRpcValue xml_container;
      if( !get(nh, param_name, xml_container, description, required) ) {
        return false;
      }

      // Make sure it's an array type
      if(xml_container.getType() != XmlRpc::XmlRpcValue::TypeArray 
          && xml_container.getType() != XmlRpc::XmlRpcValue::TypeStruct
          && xml_container.getType() != XmlRpc::XmlRpcValue::TypeString) 
      {
        std::ostringstream oss;
        oss<<"Requested parameter is not a container!"
          <<" Namespace: "<<nh.getNamespace()
          <<" Parameter: "<<param_name
          <<" Type: "<<get_xml_rpc_type_name(xml_container.getType());
        throw ros::InvalidParameterException(oss.str());
      }

      // Resize the target vector
      size = xml_container.size();

      return true;
    }

    static bool children(
        const ros::NodeHandle &nh,
        const std::string &param_name,
        std::vector<std::string> &children,
        const std::string &description,
        const bool required = false)
    {
      // Get the parameter
      XmlRpc::XmlRpcValue xml_array;
      if( !get(nh, param_name, xml_array, description, required) ) {
        return false;
      }

      // Make sure it's an array type
      if(xml_array.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        std::ostringstream oss;
        oss<<"Requested parameter is not a struct!"
          <<" Namespace: "<<nh.getNamespace()
          <<" Parameter: "<<param_name
          <<" Type: "<<get_xml_rpc_type_name(xml_array.getType());
        throw ros::InvalidParameterException(oss.str());
      }

      // Resize the target vector
      TerseXmlRpcValue terse_xml_array(xml_array);
      XmlRpc::XmlRpcValue::ValueStruct xml_map = terse_xml_array;
      children.resize(0);

      for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it=xml_map.begin();
          it != xml_map.end();
          ++it)
      {
        children.push_back(it->first);
      }

      return true;
    }


    template <class T>
      static bool require(
          const ros::NodeHandle &nh,
          const std::string &param_name,
          T &var,
          const std::string &description)
      {
        return get(nh, param_name, var, description, true);
      }
  }
}

#endif // ifndef __TERSE_ROSCPP_PARAM_H
