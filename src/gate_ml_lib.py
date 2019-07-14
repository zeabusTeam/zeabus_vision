#! /usr/bin/python2

import tensorflow as tf
from keras.models import load_model
import keras.backend
import numpy as np
import cv2
import os

class GateML:

    def __init__(self):
        keras.backend.clear_session()
        filedir = os.path.dirname(os.path.abspath(__file__))
        self.model = load_model(os.path.join(filedir, 'gate_model.h5'))
        self.graph = tf.get_default_graph()
        
    def predict(self, img):
        return self.predict_multi([img])[0]

    def predict_multi(self, imgs):
        x = []
        for img in imgs:
            histed = cv2.equalizeHist(img)
            x.append(histed)
        x = np.array(x)
        x = x.reshape(x.shape[0], 20, 40, 1)
        with self.graph.as_default():
            return self.model.predict_classes(x)

    def getprop(self, imgs):
        x = []
        for img in imgs:
            histed = cv2.equalizeHist(img)
            x.append(histed)
        x = np.array(x)
        x = x.reshape(x.shape[0], 20, 40, 1)
        with self.graph.as_default():
            return self.model.predict(x)
