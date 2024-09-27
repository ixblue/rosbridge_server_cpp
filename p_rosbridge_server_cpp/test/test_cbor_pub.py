#!/usr/bin/env python
import json
import sys
import unittest

import rostest
import rospy
from std_msgs.msg import String

from rosbridge_library.util.cbor import loads as decode_cbor

from twisted.internet import reactor
from autobahn.twisted.websocket import (WebSocketClientProtocol,
                                        WebSocketClientFactory)

from twisted.python import log
log.startLogging(sys.stderr)

TOPIC = '/b_topic'
STRING = 'B' * 10000
WARMUP_DELAY = 1.0  # seconds
TIME_LIMIT = 5.0  # seconds

"""
Subscribe with a CBOR message and receive a CBOR message
"""


class TestWebsocketCborPub(unittest.TestCase):
    def test_cbor_pub(self):
        test_client_received = []

        class TestClientProtocol(WebSocketClientProtocol):
            def onOpen(self):
                # JSON {"op": "subscribe","type": "std_msgs/String","topic": "/b_topic","compression": "cbor"}
                # converted with https://cbor.me/
                hex_string = "A4626F706973756273637269626564747970656F7374645F6D7367732F537472696E6765746F706963682F625F746F7069636B636F6D7072657373696F6E6463626F72"
                binary_message = bytes(bytearray.fromhex(hex_string))
                self.sendMessage(binary_message, isBinary=True)

            def onMessage(self, payload, is_binary):
                print("Received WS message is binary: " + str(is_binary))
                is_binary_received = is_binary
                test_client_received.append(payload)

        port = rospy.get_param('/rosbridge_websocket/actual_port')
        url = 'ws://127.0.0.1:' + str(port)

        factory = WebSocketClientFactory(url)
        factory.protocol = TestClientProtocol
        reactor.connectTCP('127.0.0.1', port, factory)

        pub = rospy.Publisher(TOPIC, String, queue_size=1)

        def publish_timer():
            rospy.sleep(WARMUP_DELAY)
            pub.publish(String(STRING))
            start = rospy.get_rostime()
            while rospy.get_rostime() < start + rospy.Duration.from_sec(TIME_LIMIT):
                rospy.sleep(0.1)
                if len(test_client_received) > 0:
                    break
            reactor.stop()
        reactor.callInThread(publish_timer)
        reactor.run()

        self.assertEqual(len(test_client_received), 1)
        websocket_message = decode_cbor(test_client_received[0])
        self.assertEqual(websocket_message['topic'], TOPIC)
        self.assertEqual(websocket_message['msg']['data'], STRING)


PKG = 'p_rosbridge_server_cpp'
NAME = 'test_websocket_cbor_pub'

if __name__ == '__main__':
    rospy.init_node(NAME)

    while not rospy.is_shutdown() and not rospy.has_param('/rosbridge_websocket/actual_port'):
        rospy.sleep(1.0)

    rostest.rosrun(PKG, NAME, TestWebsocketCborPub)
