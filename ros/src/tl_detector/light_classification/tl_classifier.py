from styx_msgs.msg import TrafficLight
import keras
from keras.models import Sequential
from keras.layers import Dense, Flatten, Lambda, Cropping2D, MaxPooling2D, Conv2D, Dropout
import os
import rospy
import tensorflow as tf
import numpy as np
import json

class TLClassifier(object):
    def __init__(self):
        base_path = os.path.dirname(os.path.abspath(__file__))
        model_file = os.path.join(base_path, 'model_sim_weights.h5')

        self.model = self.create_model()
        self.model.load_weights(model_file)
        self.graph = tf.get_default_graph()

    def create_model(self, input_shape=(600, 800, 3)):
        model = Sequential()
        
        model.add(Lambda(lambda x: x/127.5 - 1., input_shape=input_shape, output_shape=input_shape))
    
        model.add(Conv2D(32, (5,5), strides=(2,2), activation='relu'))
        model.add(Conv2D(32, (5,5), strides=(2,2), activation='relu'))
        model.add(MaxPooling2D(pool_size=(2, 2)))

        model.add(Flatten())

        model.add(Dense(128, activation='relu'))
        model.add(Dropout(0.1))


        model.add(Dense(4, activation='softmax'))
        return model

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
        image (cv::Mat): image containing the traffic light

        Returns:
        int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        with self.graph.as_default():
            logits = self.model.predict(image.reshape((1, 600, 800, 3)))
            maxindex = np.argmax(logits)


            rospy.loginfo("color: {}, logits: {}".format(maxindex, logits))
            tl = TrafficLight()
            tl.state = maxindex
            return tl.state
        return TrafficLight.UNKNOWN
