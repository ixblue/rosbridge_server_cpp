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
conn = None

if __name__ == '__main__':
    class TestClientProtocol(WebSocketClientProtocol):
        count = 0

        def onOpen(self):
            global conn
            conn = self
            print('advertising message')
            self.sendMessage(json.dumps({
                'op': 'advertise',
                'type': 'std_msgs/String',
                'topic': TOPIC,
                'latch': True,
                'compression': '',
            }).encode('utf-8'))

            self.sendMessage(json.dumps({
                'op': 'subscribe',
                'type': 'std_msgs/String',
                'topic': TOPIC,
                'compression': '',
            }).encode('utf-8'))

        def onMessage(self, payload, is_binary):
            print("Received a message")

    url = 'ws://127.0.0.1:9090'

    factory = WebSocketClientFactory(url)
    factory.protocol = TestClientProtocol
    time.sleep(1.0)
    reactor.connectTCP('127.0.0.1', 9090, factory)

    def f(s):
        global conn
        conn.sendMessage(json.dumps({
            'op': 'publish',
            'topic': TOPIC,
            'msg': {'data': 'bla bla aaaaaaaaaaaaaa'}
        }).encode('utf-8'))

    reactor.callLater(5, f, "hello, world")

    print('reactor running ...')
    reactor.run()
