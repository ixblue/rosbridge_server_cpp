#!/usr/bin/env python
import json
from websocket import create_connection

import rospy

"""
Small script to create some WS client to the bridge to stress to it

Several instances can be started at the same time
"""

PORT_PARAM = '/p_rosbridge_server_cpp/actual_port'
TOPIC = '/add_two_ints'

class WSClient():
    def __init__(self):
        msg_counter = 0

        port = rospy.get_param(PORT_PARAM)
        url = 'ws://127.0.0.1:' + str(port)
        ws = create_connection(url)
        ws.send(json.dumps({
                    'op': 'call_service',
                    'type': 'rospy_tutorials/AddTwoInts',
                    'service': TOPIC,
                    'compression': 'none',
                    'args': {'a': 1, 'b': 2}
                }).encode('utf-8'))

        result =  ws.recv()
        print('Response : ' + result)

        ws.close()

if __name__ == '__main__':
    print('Waiting for param ' + PORT_PARAM)
    while not rospy.is_shutdown() and not rospy.has_param(PORT_PARAM):
        rospy.sleep(10.0)

    print("Create a new client")
    client = WSClient()
