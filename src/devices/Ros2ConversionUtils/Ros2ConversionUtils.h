/*
 * Copyright (C) 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef ROS2_UTILS_H
#define ROS2_UTILS_H

#include <iostream>
#include <cstring>

#include <yarp/sig/all.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Property.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/header.hpp>

#include "ros2PixelCode.h"


constexpr double ONE_MILLION = 1000000;
typedef yarp::sig::ImageOf<yarp::sig::PixelFloat> DepthImage;

namespace yarp {
    namespace dev {
        namespace Ros2ConversionUtils {

    void convertTimeStampRos2ToYarp(const std_msgs::msg::Header& ros_stamp, 
                                    yarp::os::Stamp& yarp_stamp);
    void convertTimeStampYarpToRos2(const yarp::os::Stamp& yarp_stamp, 
                                    std_msgs::msg::Header& ros_stamp);

/*
    void convertImageRos2ToYarpFlexImage(sensor_msgs::msg::Image::SharedPtr ros_image, sensor_msgs::msg::CameraInfo::SharedPtr ros_camera_info, );
    void convertImageRos2ToYarpImageOf(sensor_msgs::msg::Image::SharedPtr ros_image, sensor_msgs::msg::CameraInfo::SharedPtr ros_camera_info, );
convertImageYarpFlexImagetoRos2

    void convertImageYarpFlexImagetoRos2(sensor_msgs::msg::Image::SharedPtr ros_image, sensor_msgs::msg::CameraInfo::SharedPtr ros_camera_info, );
*/
    void convertRGBImageRos2ToYarpFlexImage(sensor_msgs::msg::Image::SharedPtr ros_image_src,
                                            yarp::sig::FlexImage& dest);
    void convertDepthImageRos2ToYarpImageOf(sensor_msgs::msg::Image::SharedPtr ros_image_src,
                                            yarp::sig::ImageOf<yarp::sig::PixelFloat>& dest);

    void updateStamp(   sensor_msgs::msg::CameraInfo::SharedPtr ros_camera_info_src, 
                        std::string& frame_id_dest,
                        yarp::os::Stamp& yarp_stamp);

    void deepCopyFlexImage(const yarp::sig::FlexImage& src, yarp::sig::FlexImage& dest);

    void deepCopyImageOf(const DepthImage& src, DepthImage& dest);

/*
void deepCopyImages(const yarp::sig::FlexImage& src,
    yarp::rosmsg::sensor_msgs::Image& dest,
    const std::string& frame_id,
    const yarp::rosmsg::TickTime& timeStamp,
    const unsigned int& seq);

void deepCopyImages(const DepthImage& src,
    yarp::rosmsg::sensor_msgs::Image& dest,
    const std::string& frame_id,
    const yarp::rosmsg::TickTime& timeStamp,
    const unsigned int& seq);

void deepCopyImagesRosToYarp(const yarp::rosmsg::sensor_msgs::Image& src,
    yarp::sig::FlexImage& dest,
    std::string& frame_id,
    yarp::rosmsg::TickTime& timeStamp,
    unsigned int& seq);

void shallowCopyImages(const yarp::sig::FlexImage& src, yarp::sig::FlexImage& dest);

void shallowCopyImages(const DepthImage& src, DepthImage& dest);
*/
} // namespace RGBDRos2ConversionUtils
} // namespace dev 
} // namespace yarp

#endif //ROS2_UTILS_H
