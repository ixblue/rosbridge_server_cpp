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

import hashlib

TOPIC = '/chatter'

if __name__ == '__main__':
    class TestClientProtocol(WebSocketClientProtocol):
        count = 0

        def onOpen(self):
            self.sendMessage(json.dumps({
                'op': 'subscribe',
                'type': 'sensor_msgs/CompressedImage',
                'topic': TOPIC,
                'compression': '',
            }).encode('utf-8'))

        def onMessage(self, payload, is_binary):
            self.count = self.count + 1
            if self.count == 100:
                print("Received 100 images")
                print(hashlib.md5(payload).hexdigest())
                #print(payload)
                self.count = 0

    url = 'ws://127.0.0.1:9090'

    factory = WebSocketClientFactory(url)
    factory.protocol = TestClientProtocol
    time.sleep(3.0)
    reactor.connectTCP('127.0.0.1', 9090, factory)

    reactor.run()
