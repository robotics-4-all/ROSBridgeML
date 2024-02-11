#!/usr/bin/env python

# Copyright (C) 2020  Panayiotou, Konstantinos <klpanagi@gmail.com>
# Author: Panayiotou, Konstantinos <klpanagi@gmail.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

from typing import Any
import rospy
import time

import sys

from commlib.endpoints import endpoint_factory, EndpointType, TransportType
from commlib.transports.redis import Subscriber, ConnectionParameters

from ros_msg_transform import (
    ros_msg_to_dict,
    get_message_class,
    get_service_class,
    dict_to_ros_msg,
    dict_to_ros_srv_request,
    ros_srv_resp_to_dict
)

class B2RTopicBridge:

    def __init__(self, ros_topic: str, msg_type: Any, broker_type: Any,
                 broker_uri, broker_conn_params):
        self.ros_topic = ros_topic
        self.broker_uri = broker_uri
        self.msg_type = msg_type
        self.broker_type = broker_type
        self.broker_conn_params = broker_conn_params
        self.queue_size = 10

        self._init_ros_endpoint()
        self._init_broker_endpoint()

    def _init_ros_endpoint(self):
        self.ros_pub = rospy.Publisher(
            self.ros_topic,
            self.msg_type,
            queue_size=self.queue_size
        )
        rospy.loginfo(f'ROS Publisher <{self.ros_topic}> ready!')

    def _init_broker_endpoint(self):
        self.bsub = endpoint_factory(EndpointType.Subscriber,
                                     self.broker_type)(
            topic=self.broker_uri,
            on_message=self.on_msg,
            conn_params=self.broker_conn_params
        )
        self.bsub.run()

    def on_msg(self, data):
        _msg = dict_to_ros_msg(data, self.msg_type)
        rospy.loginfo(f'Publishing: {_msg}')
        self.ros_pub.publish(_msg)


class R2BTopicBridge:

    def __init__(self, ros_topic: str, msg_type: Any,
                 broker_type: Any, broker_uri,
                 broker_conn_params):
        self.ros_topic = ros_topic
        self.broker_uri = broker_uri
        self.msg_type = msg_type
        self.broker_type = broker_type
        self.broker_conn_params = broker_conn_params

        self._init_ros_endpoint()
        self._init_broker_endpoint()

    def _init_ros_endpoint(self):
        self.ros_sub = rospy.Subscriber(
            self.ros_topic,
            self.msg_type,
            self.on_msg
        )

    def _init_broker_endpoint(self):
        self.bpub = endpoint_factory(EndpointType.Publisher,
                                     self.broker_type)(
            topic=self.broker_uri,
            conn_params=self.broker_conn_params
        )

    def on_msg(self, msg):
        _data = ros_msg_to_dict(msg)
        rospy.loginfo(f'Publishing: {_data}')
        self.bpub.publish(_data)


def main():
    rospy.init_node('ROS2BrokerBridge')
    br_list = []
    ## Broker Connection for Bridge ------------------------------------------>
    {% if model.broker.__class__.__name__ == 'RedisBroker' %}
    broker_type = TransportType.REDIS
    from commlib.transports.redis import ConnectionParameters
    conn_params = ConnectionParameters(
        host='{{ model.broker.host }}',
        port=int({{ model.broker.port }}),
        db={{ model.broker.db }},
        username='{{ model.broker.username }}',
        password='{{ model.broker.password }}',
        ssl={{ model.broker.ssl }}
    )
    {% elif model.broker.__class__.__name__ == 'AMQPBroker' %}
    broker_type = TransportType.AMQP
    from commlib.transports.amqp import ConnectionParameters
    conn_params = ConnectionParameters(
        host='{{ model.broker.host }}',
        port={{ model.broker.port }},
        vhost='{{ model.broker.vhost }}',
        username='{{ model.broker.username }}',
        password='{{ model.broker.password }}',
        ssl={{ model.broker.ssl }}
    )
    {% elif model.broker.__class__.__name__ == 'MQTTBroker' %}
    broker_type = TransportType.MQTT
    from commlib.transports.mqtt import ConnectionParameters
    conn_params = ConnectionParameters(
        host='{{ model.broker.host }}',
        port={{ model.broker.port }},
        username='{{ model.broker.username }}',
        password='{{ model.broker.password }}',
        ssl={{ model.broker.ssl }}
    )
    {% endif %}
    
    {% for bridge in model.bridges %}
    ## <-----------------------------------------------------------------------
    {% if bridge.__class__.__name__ == 'TopicBridge' and bridge.direction == 'B2R' %}
    ## Topic Bridge B2R ----------------------------------------------------->
    from {{ bridge.msgType.split('/')[1] }}.msg import {{ bridge.msgType.split('/')[2] }}
    br = B2RTopicBridge('{{ bridge.rosURI }}', {{
        bridge.msgType.split('/')[2] }}, broker_type,
                             '{{ bridge.brokerURI }}', conn_params)
    br_list.append(br)
    ## <-----------------------------------------------------------------------
    {% elif bridge.__class__.__name__ == 'TopicBridge' and bridge.direction == 'R2B' %}
    ## Topic Bridge R2B ----------------------------------------------------->
    from {{ bridge.msgType.split('/')[1] }}.msg import {{ bridge.msgType.split('/')[2] }}
    br = R2BTopicBridge('{{ bridge.rosURI }}', {{
        bridge.msgType.split('/')[2] }}, broker_type,
                        '{{ bridge.brokerURI }}', conn_params)
    br_list.append(br)
    ## <-----------------------------------------------------------------------
    {% elif bridge.__class__.__name__ == 'ServiceBridge' and bridge.direction == 'B2R' %}
    ## RPC Bridge B2R ------------------------------------------------------->
    from {{ bridge.msgType.split('/')[1] }}.srv import {{ bridge.msgType.split('/')[2] }}
    br = B2RServiceBridge('{{ bridge.rosURI }}', {{
        bridge.msgType.split('/')[2] }}, broker_type,
                          '{{ bridge.brokerURI }}', conn_params)
    br_list.append(br)
    ## <-----------------------------------------------------------------------
    {% endif %}
    {% endfor %}
    rospy.spin()

if __name__ == "__main__":
    main()
