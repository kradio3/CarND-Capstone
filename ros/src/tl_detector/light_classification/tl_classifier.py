from styx_msgs.msg import TrafficLight
import keras
from keras.models import Model
from keras.layers import Dense, GlobalAveragePooling2D
from keras.applications.resnet50 import preprocess_input
from keras.applications.resnet50 import ResNet50, decode_predictions
import os
import tensorflow as tf
import numpy as np
import cv2

SPARSE_TO_IDX = {0:0, 1:1, 2:2, 3:4}

class TLClassifier(object):
    def __init__(self):
        base_path = os.path.dirname(os.path.abspath(__file__))
        model_file = os.path.join(base_path, 'base_rnn50.h5')

        self.model = self.create_model()
        self.model.load_weights(model_file)
        self.graph = tf.get_default_graph()


    def create_model(self):
        base_model = ResNet50(weights=None, include_top=False)
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
        x = cv2.resize(image, (224, 224)) 
        x = np.expand_dims(x, axis=0)
        x = np.float64(x)
        x = preprocess_input(x)
        with self.graph.as_default():
            logits = self.model.predict(x)
            maxindex = np.argmax(logits)
            color = SPARSE_TO_IDX[maxindex]
            tl = TrafficLight()
            tl.state = color
            return tl
