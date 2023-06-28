#!/usr/bin/env python
import json
import io
import os
import rostest
import sys
import threading
import time

from std_msgs.msg import String

from twisted.internet import reactor
from autobahn.twisted.websocket import (WebSocketClientProtocol,
                                        WebSocketClientFactory)

TOPIC = '/chatter'

if __name__ == '__main__':
    class TestClientProtocol(WebSocketClientProtocol):
        count = 0

        def onOpen(self):
            compression = os.getenv('ROSBRIDGE_COMPRESSION', '')
            print('Use compression: ', compression)
            self.sendMessage(json.dumps({
                'op': 'subscribe',
                'type': 'sensor_msgs/NavSatFix',
                'topic': TOPIC,
                'compression': compression,
            }).encode('utf-8'))

        def onMessage(self, payload, is_binary):
            self.count = self.count + 1
            if self.count == 1000:
                print("Received 1000 msgs")
                #print(payload)
                self.count = 0

    url = 'ws://127.0.0.1:9090'

    factory = WebSocketClientFactory(url)
    factory.protocol = TestClientProtocol
    time.sleep(3.0)
    reactor.connectTCP('127.0.0.1', 9090, factory)

    reactor.run()
