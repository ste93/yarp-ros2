/*
 * Copyright (C) 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_DEV_FRAMETRANSFORMGETNWCROS2_H
#define YARP_DEV_FRAMETRANSFORMGETNWCROS2_H


#include <yarp/os/Network.h>
#include <yarp/dev/IFrameTransformStorage.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IMultipleWrapper.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <frameTransformContainer.h>
#include <mutex>
#include <map>

#define ROS2NODENAME "/tfNodeGet"
#define ROS2TOPICNAME_TF "/tf"
#define ROS2TOPICNAME_TF_STATIC "/tf_static"


/*
class Ros2Init
{
public:
    Ros2Init();

    std::shared_ptr<rclcpp::Node> node;

    static Ros2Init& get();
};
*/

template <class SubPlaceHolder, class Msg1PlaceHolder, class Msg2PlaceHolder>
class DoubleSubscriber : public rclcpp::Node
{
public:
    DoubleSubscriber(std::string name, SubPlaceHolder *inputSub, std::string topic_1="", std::string topic_1="")
    : Node(name)
    {
        if(topic_1 != "")
        {
            subscription_1_ = this->create_subscription<Msg1PlaceHolder>(
                topic, 10, std::bind(&DoubleSubscriber::topic_1_callback, this, _1));
        }
        if(topic_2!= "")
        {
            subscription_2_ = this->create_subscription<Msg1PlaceHolder>(
                topic, 10, std::bind(&DoubleSubscriber::topic_2_callback, this, _1));
        }
        m_subscriber = inputSub;
    }

private:
    void topic_1_callback(const Msg1PlaceHolder msg) const
    {
        m_subscriptionTest->local_callback_1(msg);
    }
    void topic_2_callback(const Msg1PlaceHolder msg) const
    {
        m_subscriptionTest->local_callback_2(msg);
    }
    SubPlaceHolder *m_subscriber;
    rclcpp::Subscription<Msg1PlaceHolder>::SharedPtr subscription_1_;
    rclcpp::Subscription<Msg2PlaceHolder>::SharedPtr subscription_2_;
};


/**
 * @ingroup dev_impl_nwc_ros
 *
 * @brief `frameTransformGet_nwc_ros`: A ros network wrapper client that receives frame transforms from a ros topic and makes them available through an IFrameTransformStorageGet interface. See \subpage FrameTransform for additional info.
 *
 * \section FrameTransformGet_nwc_ros_device_parameters Parameters
 *
 *   Parameters required by this device are:
 * | Parameter name | SubParameter         | Type    | Units          | Default Value         | Required     | Description                                    -------            |
 * |:--------------:|:--------------------:|:-------:|:--------------:|:---------------------:|:-----------: |:-----------------------------------------------------------------:|
 * | GENERAL        |      -               | group   | -              | -                     | No           |                                                                   |
 * | -              | refresh_interval     | double  | seconds        | 0.1                   | No           | The time interval outside which timed ft will be deleted          |
 * | ROS2           |      -               | group   | -              | -                     | No           |                                                                   |
 * | -              | ft_node              | string  | -              | /tfNodeGet            | No           | The of the ROS node                                               |
 * | -              | ft_topic             | string  | -              | /tf                   | No           | The name of the ROS topic from which fts will be received         |
 * | -              | ft_topic_static      | string  | -              | /tf_static            | No           | The name of the ROS topic from which static fts will be received  |

 * **N.B.** pay attention to the difference between **tf** and **ft**
 *
 * \section FrameTransformGet_nwc_ros_device_example Example of configuration file using .ini format.
 *
 * \code{.unparsed}
 * device frameTransformGet_nwc_yarp
 * [GENERAL]
 * period 0.05
 * refresh_interval 0.2
 * [ROS]
 * ft_topic /tf
 * ft_topic_static /tf_static
 * ft_node /tfNodeGet
 * \endcode
 */


class FrameTransformGet_nwc_ros2 :
    public yarp::dev::DeviceDriver,
    public yarp::dev::IFrameTransformStorageGet,
    public yarp::os::Thread
{
public:
    ~FrameTransformGet_nwc_ros2()=default;

    //DeviceDriver
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    //Thread
    void run() override;

    //IFrameTransformStorageGet interface
    bool getTransforms(std::vector<yarp::math::FrameTransform>& transforms) const override;

    //Subscription callback
    void frameTransformTimedGet_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
    void frameTransformStaticGet_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg);

    //own
    double yarpStampFromROS2(const builtin_interfaces::msg::Time& ros2Time);
    void ros2TransformToYARP(const geometry_msgs::msg::TransformStamped& input, yarp::math::FrameTransform& output, bool isStatic);
    bool updateBuffer(const std::vector<geometry_msgs::msg::TransformStamped>& transforms, bool areStatic);
    bool setTransform(const yarp::math::FrameTransform& transform);

private:
    mutable std::mutex                                         m_trf_mutex;
    std::string                                                m_ftNodeName{ROS2NODENAME};
    std::string                                                m_ftTopic{ROS2TOPICNAME_TF};
    std::string                                                m_ftTopicStatic{ROS2TOPICNAME_TF};
    double                                                     m_refreshInterval{0.1};
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr  m_subscriptionFtTimed;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr  m_subscriptionFtStatic;
    FrameTransformContainer                                    m_ftContainer;
};

#endif // YARP_DEV_FRAMETRANSFORMGETNWCROS2_H
