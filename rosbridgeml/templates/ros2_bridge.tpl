#!/usr/bin/env python3

from typing import Any
import time

import rclpy
from rclpy.node import Node

from commlib.endpoints import endpoint_factory, EndpointType, TransportType
from commlib.transports.redis import Subscriber, ConnectionParameters

from ros2_msg_transform import (
    ros2_msg_to_dict, dict_to_ros2_msg_from_ns, dict_to_ros2_msg,
    dict_to_ros2_srv_from_ns
)


class B2RServiceBridge:

    def __init__(self, nh, ros_uri: str, msg_type: Any, broker_type: Any,
                 broker_uri, broker_conn_params):
        self.nh = nh
        self.ros_uri = ros_uri
        self.broker_uri = broker_uri
        self.msg_type = msg_type
        self.broker_type = broker_type
        self.broker_conn_params = broker_conn_params

        self._init_ros_endpoint()
        self._init_broker_endpoint()

    def _init_ros_endpoint(self):
        self.ros_client = self.nh.create_client(self.msg_type, self.ros_uri)
        tcount = 1.0
        max_tcount = 10.0
        while not self.ros_client.wait_for_service(timeout_sec=tcount):
            self.nh.get_logger().info(
                'ROS Service not available, waiting again...')
            if tcount > max_tcount:
                print('[ERROR] - ROS Service connection timeout!')
                break
            tcount += 1.0

    def _init_broker_endpoint(self):
        self.bservice = endpoint_factory(EndpointType.RPCService,
                                         self.broker_type)(
            rpc_name=self.broker_uri,
            on_request=self.on_request,
            conn_params=self.broker_conn_params
        )
        self.bservice.run()

    def on_request(self, data):
        _req = dict_to_ros2_msg(data, self.msg_type.Request)
        self.nh.get_logger().info(f'Calling Service: {self.ros_uri}')
        future = self.send_request(_req)
        resp = self.wait_for_ros_resp(future)
        _dresp = ros2_msg_to_dict(resp)
        return _dresp

    def send_request(self, req):
        future = self.cli.call_async(req)
        return future

    def wait_for_ros_resp(self, future):
        response = None
        while True:
            if future.done():
                try:
                    response = minimal_client.future.result()
                except Exception as e:
                    minimal_client.get_logger().info(
                        'Service call failed %r' % (e,))
                    response = self.msg_type.Response()
                else:
                    minimal_client.get_logger().info(
                        'Result of add_two_ints: for %d + %d = %d' %
                        (minimal_client.req.a, minimal_client.req.b, response.sum))
                break
        return response


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
    {% for bridge in model.bridges %}
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
    ## <-----------------------------------------------------------------------
    {% if bridge.__class__.__name__ == 'TopicBridge' and bridge.direction == 'B2R' %}
    ## Topic Bridge B2R ----------------------------------------------------->
    from {{ bridge.msgType.split('/')[1] }}.msg import {{ bridge.msgType.split('/')[2] }}
    br = B2RTopicBridge(nh, '{{ bridge.rosURI }}', {{
        bridge.msgType.split('/')[2] }}, broker_type,
                             '{{ bridge.brokerURI }}', conn_params)
    br_list.append(br)
    ## <-----------------------------------------------------------------------
    {% elif bridge.__class__.__name__ == 'TopicBridge' and bridge.direction == 'R2B' %}
    ## Topic Bridge R2B ----------------------------------------------------->
    from {{ bridge.msgType.split('/')[1] }}.msg import {{ bridge.msgType.split('/')[2] }}
    br = R2BTopicBridge(nh, '{{ bridge.rosURI }}', {{
        bridge.msgType.split('/')[2] }}, broker_type,
                             '{{ bridge.brokerURI }}', conn_params)
    br_list.append(br)
    ## <-----------------------------------------------------------------------
    {% elif bridge.__class__.__name__ == 'ServiceBridge' and bridge.direction == 'B2R' %}
    ## RPC Bridge B2R ------------------------------------------------------->
    from {{ bridge.msgType.split('/')[1] }}.srv import {{ bridge.msgType.split('/')[2] }}
    br = B2RServiceBridge(nh, '{{ bridge.rosURI }}', {{
        bridge.msgType.split('/')[2] }}, broker_type,
                             '{{ bridge.brokerURI }}', conn_params)
    br_list.append(br)
    ## <-----------------------------------------------------------------------
    {% endif %}
    {% endfor %}

    rclpy.spin(nh)
