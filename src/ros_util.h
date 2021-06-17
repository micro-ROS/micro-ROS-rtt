// Copyright (c) 2021 - for information on the respective copyright owner
// see the NOTICE file and/or the repository https://github.com/micro-ROS/micro-ROS-rtt.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef __ROS_UTIL_H
#define __ROS_UTIL_H

#include <exception>
#include <string>
#include <rcl/error_handling.h>
#include <syslog.h>

namespace uros {
namespace rtt {

class RCLException : public std::exception {
public:
    RCLException() : _msg(rcutils_get_error_string().str) {
        rcutils_reset_error();
    }
    RCLException(const std::string& msg) : _msg(msg) {        
    }

    virtual const char* what() const noexcept override {
        return _msg.c_str();
    }
private:
    const std::string _msg;
};

} // namespace rtt
} // namespace uros

// TODO replace this with calls to rcutil_logging
#define ROS_DEBUG(fmt, ...)     /*syslog(LOG_DEBUG, fmt, __VA_ARGS__);*/
#define ROS_INFO(fmt, ...)      fprintf(stdout, fmt, __VA_ARGS__);
#define ROS_ERROR(fmt, ...)     fprintf(stderr, fmt, __VA_ARGS__);

#define PRINT_RCL_ERROR(func) \
  do { \
    ROS_ERROR("error in " #func ": %s\n", rcutils_get_error_string().str); \
    rcl_reset_error(); \
  } while (0)

#define CHECK_RET(FUNC) { \
    rcl_ret_t macro_rc = FUNC ; \
    if(macro_rc != RMW_RET_OK) { \
        PRINT_RCL_ERROR(FUNC); \
        return -1; \
    } else { \
        /* fprintf(stdout, "OK on " #FUNC "\n"); */\
    } \
}
#define WARN_RET(FUNC) { rcl_ret_t macro_rc = FUNC ; if(macro_rc != RMW_RET_OK) { PRINT_RCL_ERROR(FUNC); } }
#define THROW_RET(FUNC) { rcl_ret_t macro_rc = FUNC ; if(macro_rc != RMW_RET_OK) { throw RCLException(rcutils_get_error_string().str); } }

#endif
