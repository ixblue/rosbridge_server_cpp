#!/usr/bin/env python
import json
from websocket import create_connection

import rospy

"""
Small script to create some WS client to the bridge to stress to it

Several instances can be started at the same time
"""

PORT_PARAM = '/p_rosbridge_server_cpp/actual_port'
TOPIC = '/listener'

class WSClient():
    def __init__(self):
        msg_counter = 0

        port = rospy.get_param(PORT_PARAM)
        url = 'ws://127.0.0.1:' + str(port)
        ws = create_connection(url)
        ws.send(json.dumps({
                    'op': 'subscribe',
                    'type': 'std_msgs/String',
                    'topic': TOPIC,
                    'compression': 'none',
                }).encode('utf-8'))

        while msg_counter < 10:
            result =  ws.recv()
            # Add a sleep here to fill the WS buffers and
            # generate an abort on the rosbridge side
            rospy.sleep(1.0)
            msg_counter += 1
        ws.close()

if __name__ == '__main__':
    print('Waiting for param ' + PORT_PARAM)
    while not rospy.is_shutdown() and not rospy.has_param(PORT_PARAM):
        rospy.sleep(10.0)

    while not rospy.is_shutdown():
        # Create new connections in loop
        print("Create a new client")
        client = WSClient()
