#!/usr/bin/env python3

from typing import Any
import time

import rclpy
from rclpy.node import Node

from commlib.endpoints import endpoint_factory, EndpointType, TransportType
from commlib.transports.redis import Subscriber, ConnectionParameters

from ros2_msg_conv import (
    ros2_msg_to_dict, dict_to_ros2_msg_from_ns, dict_to_ros2_msg,
    dict_to_ros2_srv_from_ns
)


class B2RTopicBridge:

    def __init__(self, nh, ros_topic: str, msg_type: Any, broker_type: Any,
                 broker_uri, broker_conn_params):
        self.nh = nh
        self.ros_topic = ros_topic
        self.broker_uri = broker_uri
        self.msg_type = msg_type
        self.broker_type = broker_type
        self.broker_conn_params = broker_conn_params

        self._init_ros_endpoint()
        self._init_broker_endpoint()

    def _init_ros_endpoint(self):
        self.ros_pub = self.nh.create_publisher(self.msg_type, self.ros_topic, 10)

    def _init_broker_endpoint(self):
        self.bsub = endpoint_factory(EndpointType.Subscriber,
                                     self.broker_type)(
            topic=self.broker_uri,
            on_message=self.on_msg,
            conn_params=self.broker_conn_params
        )
        self.bsub.run()

    def on_msg(self, data):
        _msg = dict_to_ros2_msg(data, self.msg_type)
        self.nh.get_logger().info('Publishing: "%s"' % _msg)
        self.ros_pub.publish(_msg)


class R2BTopicBridge:

    def __init__(self, nh, ros_topic: str, msg_type: Any, broker_type: Any,
                 broker_uri, broker_conn_params):
        self.nh = nh
        self.ros_topic = ros_topic
        self.broker_uri = broker_uri
        self.msg_type = msg_type
        self.broker_type = broker_type
        self.broker_conn_params = broker_conn_params

        self._init_ros_endpoint()
        self._init_broker_endpoint()

    def _init_ros_endpoint(self):
        self.nh.ros_sub = self.nh.create_subscription(
            self.msg_type,
            self.ros_topic,
            self.on_msg,
            10)

    def _init_broker_endpoint(self):
        self.bpub = endpoint_factory(EndpointType.Publisher,
                                     self.broker_type)(
            topic=self.broker_uri,
            conn_params=self.broker_conn_params
        )

    def on_msg(self, msg):
        _data = ros2_msg_to_dict(msg)
        self.nh.get_logger().info('Publishing: "%s"' % _data)
        self.bpub.publish(_data)


if __name__ == "__main__":
    rclpy.init()
    nh = Node('ROSBridge')
    br_list = []
    {% for bridge in bridges %}
    {% if bridge.brokerConn.__class__.__name__ == 'RedisConnection' %}
    broker_type = TransportType.REDIS
    from commlib.transports.redis import ConnectionParameters, Credentials
    creds = Credentials(username='{{ bridge.brokerConn.username }}',
                        password='{{ bridge.brokerConn.password }}')
    conn_params = ConnectionParameters(host='{{ bridge.brokerConn.host }}',
                                       port=int({{ bridge.brokerConn.port }}),
                                       db=int({{ bridge.brokerConn.db }}),
                                       creds=creds)
    {% elif bridge.brokerConn.__class__.__name__ == 'AMQPConnection' %}
    broker_type = TransportType.AMQP
    from commlib.transports.amqp import ConnectionParameters, Credentials
    creds = Credentials(username='{{ bridge.brokerConn.username }}',
                        password='{{ bridge.brokerConn.password }}')
    conn_params = ConnectionParameters(host='{{ bridge.brokerConn.host }}',
                                       port=int({{ bridge.brokerConn.port }}),
                                       vhost='{{ bridge.brokerConn.vhost }}',
                                       creds=creds)
    {% elif bridge.brokerConn.__class__.__name__ == 'MQTTConnection' %}
    broker_type = TransportType.MQTT
    from commlib.transports.mqtt import ConnectionParameters, Credentials
    creds = Credentials(username='{{ bridge.brokerConn.username }}',
                        password='{{ bridge.brokerConn.password }}')
    conn_params = ConnectionParameters(host='{{ bridge.brokerConn.host }}',
                                       port=int({{ bridge.brokerConn.port }}),
                                       creds=creds)
    {% endif %}
    from {{ bridge.msgType.split('/')[0] }}.msg import {{ bridge.msgType.split('/')[1] }}
    {% if bridge.__class__.__name__ == 'TopicBridge' and bridge.direction == 'B2R' %}
    br = B2RTopicBridge(nh, '{{ bridge.rosURI }}', {{
        bridge.msgType.split('/')[1] }}, broker_type,
                        '{{ bridge.brokerURI }}', conn_params)
    br_list.append(br)
    {% elif bridge.__class__.__name__ == 'TopicBridge' and bridge.direction == 'R2B' %}
    br = R2BTopicBridge(nh, '{{ bridge.rosURI }}', {{
        bridge.msgType.split('/')[1] }}, broker_type,
                        '{{ bridge.brokerURI }}', conn_params)
    br_list.append(br)
    {% endif %}
    {% endfor %}

    rclpy.spin(nh)
