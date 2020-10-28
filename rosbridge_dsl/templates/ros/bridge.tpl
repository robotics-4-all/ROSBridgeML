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

{% for c in bridges['topic']['b2r'] %}
    broker = BrokerDefinition(
        name='{{ c.brokerConn.brokerRef.name }}',
        host='{{ c.brokerConn.brokerRef.host }}',
        port='{{ c.brokerConn.brokerRef.amqp_port }}',
        vhost='{{ c.brokerConn.vhost }}',
    )
    ros_ep_{{ c.name }} = ROSSubEndpoint(
        msg_type='{{ c.msgType }}',
        uri='{{ c.rosURI }}',
        name='{{ c.name }}'
    )

    broker_ep_{{ c.name }} = BrokerSubEndpoint(
        uri='{{ c.brokerURI }}',
        name='{{ c.name }}',
        broker_ref=broker,
        auth=BrokerAuthPlain(username='{{ c.brokerConn.username }}', password='{{ c.brokerConn.password }}')
    )

    executor.run_connector(SubConnector(ros_ep_{{ c.name }}, broker_ep_{{ c.name }}))
{% endfor %}
{% for c in bridges['topic']['r2b'] %}
    broker = BrokerDefinition(
        name='{{ c.brokerConn.brokerRef.name }}',
        host='{{ c.brokerConn.brokerRef.host }}',
        port='{{ c.brokerConn.brokerRef.amqp_port }}',
        vhost='{{ c.brokerConn.vhost }}',
    )
    ros_ep_{{ c.name }} = ROSPubEndpoint(
        msg_type='{{ c.msgType }}',
        uri='{{ c.rosURI }}',
        name='{{ c.name }}'
    )

    broker_ep_{{ c.name }} = BrokerPubEndpoint(
        uri='{{ c.brokerURI }}',
        name='{{ c.name }}',
        broker_ref=broker,
        auth=BrokerAuthPlain(username='{{ c.brokerConn.username }}', password='{{ c.brokerConn.password }}')
    )

    executor.run_connector(PubConnector(ros_ep_{{ c.name }}, broker_ep_{{ c.name }}))
{% endfor %}
{% for c in bridges['rpc'] %}
    ros_ep_{{ c.name }} = ROSServiceEndpoint(
        srv_type='{{ c.msgType }}',
        uri='{{ c.rosURI }}',
        name='{{ c.name }}'
    )

    broker_ep_{{ c.name }} = BrokerRPCEndpoint(
        uri='{{ c.brokerURI }}',
        name='{{ c.name }}',
        broker_ref=broker,
        auth=BrokerAuthPlain(username='{{ c.brokerConn.username }}', password='{{ c.brokerConn.password }}')
    )

    executor.run_connector(RPCConnector(ros_ep_{{ c.name }}, broker_ep_{{ c.name }}))
{% endfor %}
    executor.run_forever()

if __name__ == "__main__":
    main()
