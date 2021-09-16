# Copyright (C) 2006-2021 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

#if(NOT YARP_COMPILE_DEVICE_PLUGINS)
#  return()
#endif()
find_package(rclcpp REQUIRED)

add_library(Ros2Subscriber SHARED)

target_sources(Ros2Subscriber PRIVATE Ros2Subscriber.h)
target_include_directories(Ros2Subscriber PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(Ros2Subscriber PRIVATE
                                 rclcpp::rclcpp)

set_property(TARGET Ros2Subscriber PROPERTY FOLDER "Libraries/Msgs")