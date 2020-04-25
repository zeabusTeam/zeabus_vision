#!/usr/bin/python2.7
"""
    File name: example-subscriber.py
    Author: AyumiizZ
    Date created: 2020/04/25
    Python Version: 2.7
    how to run: python example-subscriber.py # this is standalone version
"""
import zbuspy
import time
import cv2 as cv
import zmq
import numpy as np

image = None


def callback(msg):
    global image
    image = msg


def main():
    while True:
        try:
            if image is None:
                continue
            cv.imshow('images', image)
            cv.waitKey(1)

        except KeyboardInterrupt:
            cv.destroyAllWindows()
            print ("\n\nBye bye\n")
            break
        except:
            print ("Unexpected error")
            break


if __name__ == "__main__":
    # zbuspy.Subscriber(image.topic.FRONT, CompressedImage,
    #  callback=image.callback, queue_size=3)
    ip = '192.168.1.122'
    port = 6969
    sub = zbuspy.Subscriber(
        ip=ip, port=port, datatype=zbuspy.msgs.OPENCVIMAGE, callback=callback)
    main()
    sub.kill()
