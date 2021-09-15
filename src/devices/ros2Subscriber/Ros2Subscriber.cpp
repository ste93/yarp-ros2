/*
 * Copyright (C) 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "Ros2Subscriber.h"
#include <yarp/os/LogStream.h>

#include <yarp/os/LogComponent.h>

//template <class CallbackClass, typename MsgClass>
//Ros2Subscriber<CallbackClass, MsgClass>::Ros2Subscriber(rclcpp::Node::SharedPtr node, CallbackClass* callbackClass)
//{
//    yInfo() << "simple subscrriber starting ";
//    m_node = node;
//    m_callbackClass = callbackClass;
//}
//
//template <class CallbackClass, typename MsgClass>
//void Ros2Subscriber<CallbackClass, MsgClass>::subscribe_to_topic(std::string topic_name) {
//    yInfo() << "simple subscrriber creating topic " << topic_name;
//    m_subscription.push_back(m_node->create_subscription<MsgClass>(
//        topic_name,
//        10,
//        [this, topic_name](const typename MsgClass::SharedPtr msg) {
//            m_callbackClass->callback(msg, topic_name);
//        }));
//}