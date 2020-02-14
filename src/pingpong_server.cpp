// Copyright (c) 2020 Robert Bosch GmbH
// Author: Ingo Luetkebohle <ingo.luetkebohle@de.bosch.com>
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.

// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.

// * Neither the name of thin_drivers nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <rcl/rcl.h>
#include <std_msgs/msg/header.h>
#include "ros_util.h"

using namespace uros::rtt;

class PongServer
{
public:
  PongServer(int argc, char* argv[])
      : context(rcl_get_zero_initialized_context())
      , node(rcl_get_zero_initialized_node())
      , publisher(rcl_get_zero_initialized_publisher())
      , subscriber(rcl_get_zero_initialized_subscription())
      , wait_set(rcl_get_zero_initialized_wait_set())
  {
    // RCL NODE INITIALIZATION
    rcl_ret_t rc = RMW_RET_OK;

    ROS_INFO("pong_server: %s\n", "Creating context");
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    THROW_RET(rcl_init_options_init(&init_options, rcl_get_default_allocator()));    
    THROW_RET(rcl_init(argc, argv, &init_options, &context));
  
    ROS_INFO("pong_server: %s\n", "Creating node");
    rcl_node_options_t node_ops = rcl_node_get_default_options();
    THROW_RET(rcl_node_init(&node, "pong_server", "", &context, &node_ops));
    // END RCL NODE INIT

    // COMMUNICATION INIT
    ROS_INFO("pong_server: %s\n", "Creating publisher");
    const rosidl_message_type_support_t *pub_ts = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header);
    rcl_publisher_options_t pub_opt = rcl_publisher_get_default_options();

    rc = rcl_publisher_init(
        &publisher,
        &node,
        pub_ts,
        "uros_pong",
        &pub_opt);

    if (rc != RCL_RET_OK) {
        RCLException ex;        
        WARN_RET(rcl_node_fini( &node))
        throw ex;
    }

    ROS_INFO("pong_server: %s\n", "Creating subscriber");
    rcl_subscription_options_t subscription_ops = rcl_subscription_get_default_options();
    const rosidl_message_type_support_t *sub_ts = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header);
  
    rc = rcl_subscription_init(
        &subscriber,
        &node,
        sub_ts,
        "uros_ping",
        &subscription_ops);
    if(rc != RCL_RET_OK) {
        rcutils_reset_error();
        WARN_RET(rcl_publisher_fini( &publisher, &node))
        WARN_RET(rcl_node_fini( &node))
        throw RCLException("Failed to create ping subscriber");
    }

    ROS_INFO("pong_server: %s\n", "Creating waitset");
    rc = rcl_wait_set_init(&wait_set, 1, 0, 0, 0, 0, 0, &context, rcl_get_default_allocator());
    if (rc != RCL_RET_OK) {
        rcutils_reset_error();
        WARN_RET(rcl_subscription_fini( &subscriber, &node))
        WARN_RET(rcl_publisher_fini( &publisher, &node))
        WARN_RET(rcl_node_fini( &node))
        throw RCLException("Failed to create wait set");
    }

    if(!std_msgs__msg__Header__init(&sub_msg)) {
        WARN_RET(rcl_subscription_fini( &subscriber, &node))
        WARN_RET(rcl_publisher_fini( &publisher, &node))
        WARN_RET(rcl_node_fini( &node))
        throw RCLException("Failed to initialize message");
    }
    const size_t BUFSIZE = 1024;
    void * data = realloc(sub_msg.frame_id.data, BUFSIZE);
    bool result = false;
    if(data != NULL) {
        sub_msg.frame_id.data = (char*)data;
        sub_msg.frame_id.capacity = BUFSIZE;
    } else {
        std_msgs__msg__Header__fini(&sub_msg);
        WARN_RET(rcl_subscription_fini( &subscriber, &node))
        WARN_RET(rcl_publisher_fini( &publisher, &node))
        WARN_RET(rcl_node_fini( &node))
        throw RCLException("Failed to reallocate message buffer");
    }
  }

  ~PongServer()
  {
    std_msgs__msg__Header__fini(&sub_msg);
    WARN_RET(rcl_subscription_fini( &subscriber, &node))
    WARN_RET(rcl_publisher_fini( &publisher, &node))
    WARN_RET(rcl_node_fini( &node))
  }

  bool wait(uint32_t timeout_ms)
  {
    rcl_ret_t rc = RCL_RET_OK;
    // set rmw fields to NULL
    THROW_RET(rcl_wait_set_clear(&wait_set))

    size_t index = 0; // is never used - denotes the index of the subscription in the storage container
    THROW_RET(rcl_wait_set_add_subscription(&wait_set, &subscriber, &index))
  
    rc = rcl_wait(&wait_set, RCL_MS_TO_NS(timeout_ms));
    if (rc  == RCL_RET_TIMEOUT) {
        return false; // doing nothing
    }					

    if (rc != RCL_RET_OK) {
        PRINT_RCL_ERROR(rcl_wait);
        return false;
    }

    // if RET_OK => one subscription is received because only ONE subscription has been added
    // just double check here.
    if ( wait_set.subscriptions[0] ){
        rc = rcl_take(&subscriber, &sub_msg, &messageInfo, NULL);
        if(rc == RCL_RET_OK) {
            // just re-publish it
            rcl_publish(&publisher, &sub_msg, NULL);
            return true;
        }

        if(rc != RCL_RET_OK) {
            if(rc != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
                PRINT_RCL_ERROR(rcl_take);
                return false;
            }
        }
        
        return false;
    } else {
        //sanity check
        fprintf(stderr, "[spin_node_once] no subscription received.\n");
    }

    return false;
  }
private:
  rcl_context_t context;
  rcl_node_t node;
  rcl_publisher_t publisher;
  rcl_subscription_t subscriber;
  rcl_wait_set_t wait_set;
  rmw_message_info_t messageInfo;
  std_msgs__msg__Header sub_msg;
};


int main(int argc, char** argv)
{
  PongServer pong_server(argc, argv);
  while (true)
  {
      pong_server.wait(1000);
  }
  return 0;
}