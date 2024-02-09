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

from __future__ import (
    absolute_import,
    division,
    print_function,
    unicode_literals
)

import sys

from ros2broker import (
    PubConnector, ROSPubEndpoint, BrokerPubEndpoint,
    SubConnector, ROSSubEndpoint, BrokerSubEndpoint,
    RPCConnector, ROSServiceEndpoint, BrokerRPCEndpoint,
    BrokerAuthPlain, BrokerDefinition,
    ConnectorThreadExecutor
)


def main():
    executor = ConnectorThreadExecutor()
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
    
    broker = BrokerDefinition(
        name='{{ model.broker.name }}',
        host='{{ model.broker.host }}',
        port={{ model.broker.port }},
        username='{{ model.broker.username }}',
        password='{{ model.broker.password }}',
        ssl='{{ model.broker.ssl }}',
    )
    
    {% for bridge in model.bridges %}
    {% if bridge.__class__.__name__ == 'TopicBridge' and bridge.direction == 'B2R' %}
    ros_ep_{{ bridge.name }} = ROSSubEndpoint(
        msg_type='{{ bridge.msgType }}',
        uri='{{ bridge.rosURI }}',
        name='{{ bridge.name }}'
    )
    broker_ep_{{ bridge.name }} = BrokerSubEndpoint(
        uri='{{ bridge.brokerURI }}',
        name='{{ bridge.name }}',
        broker_ref=broker
    )
    executor.run_connector(SubConnector(ros_ep_{{ bridge.name }}, broker_ep_{{ bridge.name }}))
    {% elif bridge.__class__.__name__ == 'TopicBridge' and bridge.direction == 'R2B' %}
    ros_ep_{{ bridge.name }} = ROSPubEndpoint(
        msg_type='{{ bridge.msgType }}',
        uri='{{ bridge.rosURI }}',
        name='{{ bridge.name }}'
    )
    broker_ep_{{ bridge.name }} = BrokerPubEndpoint(
        uri='{{ bridge.brokerURI }}',
        name='{{ bridge.name }}',
        broker_ref=broker
    )
    executor.run_connector(PubConnector(ros_ep_{{ bridge.name }}, broker_ep_{{ bridge.name }}))
    {% elif bridge.__class__.__name__ == 'ServiceBridge' and bridge.direction == 'B2R' %}
    ros_ep_{{ bridge.name }} = ROSServiceEndpoint(
        srv_type='{{ bridge.msgType }}',
        uri='{{ bridge.rosURI }}',
        name='{{ bridge.name }}'
    )
    broker_ep_{{ bridge.name }} = BrokerRPCEndpoint(
        uri='{{ bridge.brokerURI }}',
        name='{{ bridge.name }}',
        broker_ref=broker
    )
    executor.run_connector(RPCConnector(ros_ep_{{ bridge.name }}, broker_ep_{{ bridge.name }}))
    {% endfor %}
    executor.run_forever()

if __name__ == "__main__":
    main()
