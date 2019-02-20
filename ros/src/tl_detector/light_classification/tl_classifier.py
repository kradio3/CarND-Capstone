from styx_msgs.msg import TrafficLight
import keras
from keras.models import Model
from keras.layers import Dense, GlobalAveragePooling2D
from keras.applications.vgg16 import preprocess_input
from keras.applications.vgg16 import VGG16
import os
import rospy
import tensorflow as tf
import numpy as np
import json
import cv2
from time import sleep

SPARSE_TO_IDX = {0:0, 1:1, 2:2, 3:4}
C_TO_COLOR = {0:'RED', 1:'YELLOW', 2:'GREEN', 4: 'UNDEFINED'}

class TLClassifier(object):
    def __init__(self):
        base_path = os.path.dirname(os.path.abspath(__file__))
        model_file = os.path.join(base_path, 'base_vgg16.h5')

        self.model = self.create_model()
        self.model.load_weights(model_file)
        self.graph = tf.get_default_graph()


    def create_model(self):
        base_model = VGG16(weights='imagenet', include_top=False)
        for layer in base_model.layers:
            layer.trainable = False
                
        x = base_model.output
        x = GlobalAveragePooling2D()(x)
        predictions = Dense(4, activation='softmax')(x)
        model = Model(inputs=base_model.input, outputs=predictions)
        return model

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
        image (cv::Mat): image containing the traffic light

        Returns:
        int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        x = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        x = cv2.resize(x, (224, 224)) 
        x = np.expand_dims(x, axis=0)
        x = np.float64(x)
        x = preprocess_input(x)
        with self.graph.as_default():
            logits = self.model.predict(x)
            maxindex = np.argmax(logits)
            color = SPARSE_TO_IDX[maxindex]
            #rospy.loginfo("color: {}, logits: {}".format(C_TO_COLOR[color], logits))
            tl = TrafficLight()
            tl.state = color
            return tl
