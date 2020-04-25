#!/usr/bin/python2.7
"""
    File name: zbuspy.py
    Author: AyumiizZ
    Date created: 2020/04/25
    Python Version: 2.7
"""
import cv2 as cv
import zmq
import threading
import numpy as np
import time


class msgs:
    OPENCVIMAGE = 1
    JSON = 2


class Publisher:
    """
        for enable_sync must enable for pub and sub
    """

    def __init__(self, port, datatype, enable_sync=False):
        self.ip = '0.0.0.0'
        self.port = port
        self.datatype = datatype
        self.enable_sync = enable_sync
        self.bind_to = 'tcp://{}:{}'.format(self.ip, self.port)
        self.ctx = zmq.Context()
        self.socket = self.ctx.socket(zmq.PUB)
        # self.socket.setsockopt(zmq.SNDBUF, 10) # set buffer size = 10
        # self.socket.setsockopt(zmq.CONFLATE, 1) # sent only last message
        self.socket.bind(self.bind_to)

    def sync(self):
        # use bind socket + 1
        sync_with = 'tcp://{}:{}'.format(self.ip, int(self.port)+1)
        ctx = zmq.Context.instance()
        s = ctx.socket(zmq.REP)
        s.bind(sync_with)
        print("Waiting for subscriber to connect...")
        s.recv()
        print("Sync Done.")
        s.send_string('GO')

    def publish(self, data):
        if self.enable_sync:
            sync()

        if self.datatype == msgs.OPENCVIMAGE:
            encoded, img_encoded = cv.imencode('.jpg', data)
            self.socket.send(img_encoded)
        elif self.datatype == msgs.JSON:
            self.socket.send(data)
        else:
            print("Now support only json and opencv images")


class Subscriber:
    """
        for enable_sync must enable for pub and sub
    """

    def __init__(self, ip, port, datatype, callback, callback_args=None, enable_sync=False):
        self.ip = ip
        self.port = port
        self.datatype = datatype
        self.enable_sync = enable_sync

        self.connect_to = 'tcp://{}:{}'.format(self.ip, self.port)
        self.ctx = zmq.Context()
        self.socket = self.ctx.socket(zmq.SUB)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, '')
        self.socket.setsockopt(zmq.LINGER, 0)
        # self.socket.setsockopt(zmq.RCVTIMEO, 10)
        # self.socket.setsockopt(zmq.RCVBUF, 10) # set buffer size = 10
        # self.socket.setsockopt(zmq.CONFLATE, 1) # use only last message
        self.socket.connect(self.connect_to)
        if self.enable_sync:
            self.sync()
        self.killed = False
        self.sub_thread = threading.Thread(
            target=self.subscribe, args=(callback, callback_args))
        self.sub_thread.start()
        

    def sync(self):
        sync_with = 'tcp://{}:{}'.format(self.ip, int(self.port)+1)
        ctx = zmq.Context.instance()
        s = ctx.socket(zmq.REQ)
        s.connect(sync_with)
        s.send('READY')
        s.recv()

    def subscribe(self, callback, callback_args=None):
        while not self.killed:
            try:
                data = self.socket.recv()
            except Exception:
                print('Recieving End')
                break
            if self.datatype == msgs.OPENCVIMAGE:
                numpy_image = np.fromstring(data, dtype=np.uint8)
                cv_image = cv.imdecode(numpy_image, cv.IMREAD_COLOR)
                if callback_args is None:
                    callback(cv_image)
                else:
                    callback(cv_image, callback_args)

    def kill(self):
        self.killed = True
        self.socket.close()
        self.ctx.term()
