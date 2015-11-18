/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Auto-generated by gensrv_cpp from file /home/mfiore/catkin_ws/src/appl/srv/appl_request.srv
 *
 */


#ifndef APPL_MESSAGE_APPL_REQUEST_H
#define APPL_MESSAGE_APPL_REQUEST_H

#include <ros/service_traits.h>


#include <appl/appl_requestRequest.h>
#include <appl/appl_requestResponse.h>


namespace appl
{

struct appl_request
{

typedef appl_requestRequest Request;
typedef appl_requestResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct appl_request
} // namespace appl


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::appl::appl_request > {
  static const char* value()
  {
    return "56c6280feb8a8eace7add2e5e377b170";
  }

  static const char* value(const ::appl::appl_request&) { return value(); }
};

template<>
struct DataType< ::appl::appl_request > {
  static const char* value()
  {
    return "appl/appl_request";
  }

  static const char* value(const ::appl::appl_request&) { return value(); }
};


// service_traits::MD5Sum< ::appl::appl_requestRequest> should match 
// service_traits::MD5Sum< ::appl::appl_request > 
template<>
struct MD5Sum< ::appl::appl_requestRequest>
{
  static const char* value()
  {
    return MD5Sum< ::appl::appl_request >::value();
  }
  static const char* value(const ::appl::appl_requestRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::appl::appl_requestRequest> should match 
// service_traits::DataType< ::appl::appl_request > 
template<>
struct DataType< ::appl::appl_requestRequest>
{
  static const char* value()
  {
    return DataType< ::appl::appl_request >::value();
  }
  static const char* value(const ::appl::appl_requestRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::appl::appl_requestResponse> should match 
// service_traits::MD5Sum< ::appl::appl_request > 
template<>
struct MD5Sum< ::appl::appl_requestResponse>
{
  static const char* value()
  {
    return MD5Sum< ::appl::appl_request >::value();
  }
  static const char* value(const ::appl::appl_requestResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::appl::appl_requestResponse> should match 
// service_traits::DataType< ::appl::appl_request > 
template<>
struct DataType< ::appl::appl_requestResponse>
{
  static const char* value()
  {
    return DataType< ::appl::appl_request >::value();
  }
  static const char* value(const ::appl::appl_requestResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // APPL_MESSAGE_APPL_REQUEST_H