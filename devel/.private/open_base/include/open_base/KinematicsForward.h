// Generated by gencpp from file open_base/KinematicsForward.msg
// DO NOT EDIT!


#ifndef OPEN_BASE_MESSAGE_KINEMATICSFORWARD_H
#define OPEN_BASE_MESSAGE_KINEMATICSFORWARD_H

#include <ros/service_traits.h>


#include <open_base/KinematicsForwardRequest.h>
#include <open_base/KinematicsForwardResponse.h>


namespace open_base
{

struct KinematicsForward
{

typedef KinematicsForwardRequest Request;
typedef KinematicsForwardResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct KinematicsForward
} // namespace open_base


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::open_base::KinematicsForward > {
  static const char* value()
  {
    return "4f057007e926424c383ecaa1817c5251";
  }

  static const char* value(const ::open_base::KinematicsForward&) { return value(); }
};

template<>
struct DataType< ::open_base::KinematicsForward > {
  static const char* value()
  {
    return "open_base/KinematicsForward";
  }

  static const char* value(const ::open_base::KinematicsForward&) { return value(); }
};


// service_traits::MD5Sum< ::open_base::KinematicsForwardRequest> should match
// service_traits::MD5Sum< ::open_base::KinematicsForward >
template<>
struct MD5Sum< ::open_base::KinematicsForwardRequest>
{
  static const char* value()
  {
    return MD5Sum< ::open_base::KinematicsForward >::value();
  }
  static const char* value(const ::open_base::KinematicsForwardRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::open_base::KinematicsForwardRequest> should match
// service_traits::DataType< ::open_base::KinematicsForward >
template<>
struct DataType< ::open_base::KinematicsForwardRequest>
{
  static const char* value()
  {
    return DataType< ::open_base::KinematicsForward >::value();
  }
  static const char* value(const ::open_base::KinematicsForwardRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::open_base::KinematicsForwardResponse> should match
// service_traits::MD5Sum< ::open_base::KinematicsForward >
template<>
struct MD5Sum< ::open_base::KinematicsForwardResponse>
{
  static const char* value()
  {
    return MD5Sum< ::open_base::KinematicsForward >::value();
  }
  static const char* value(const ::open_base::KinematicsForwardResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::open_base::KinematicsForwardResponse> should match
// service_traits::DataType< ::open_base::KinematicsForward >
template<>
struct DataType< ::open_base::KinematicsForwardResponse>
{
  static const char* value()
  {
    return DataType< ::open_base::KinematicsForward >::value();
  }
  static const char* value(const ::open_base::KinematicsForwardResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // OPEN_BASE_MESSAGE_KINEMATICSFORWARD_H
