from styx_msgs.msg import TrafficLight
from keras.models import Model
from keras.layers import Dense, GlobalAveragePooling2D
from keras.applications.resnet50 import ResNet50, preprocess_input
import os
import tensorflow as tf
import numpy as np
import cv2
import rospy

SPARSE_TO_IDX = {0:0, 1:1, 2:2, 3:4}
MODEL_PICTURE_SIZE = (224, 224)

class TLClassifier(object):
    def __init__(self, model_weights_file):
        rospy.loginfo(model_weights_file)
        base_path = os.path.dirname(os.path.abspath(__file__))
        model_weights = os.path.join(base_path, model_weights_file)
        self.model = self.create_model()
        self.model.load_weights(model_weights)
        self.graph = tf.get_default_graph()

    def create_model(self):
        base_model = ResNet50(weights=None, include_top=False)
        x = base_model.output
        x = GlobalAveragePooling2D()(x)
        predictions = Dense(4, activation='softmax')(x)
        model = Model(inputs=base_model.input, outputs=predictions)
        return model

    def get_classification(self, image):
        x = cv2.resize(image, MODEL_PICTURE_SIZE) 
        x = np.expand_dims(x, axis=0)
        x = np.float64(x)
        x = preprocess_input(x)
        with self.graph.as_default():
            logits = self.model.predict(x)
            maxindex = np.argmax(logits)
            color = SPARSE_TO_IDX[maxindex]
            rospy.loginfo('COLOR {}'.format(color))
            tl = TrafficLight()
            tl.state = color
            return tl
