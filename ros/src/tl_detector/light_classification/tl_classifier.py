from styx_msgs.msg import TrafficLight
from keras.models import Model
from keras.layers import Dense, GlobalAveragePooling2D
from keras.applications.resnet50 import ResNet50, preprocess_input
import os
import tensorflow as tf
import numpy as np
import cv2

SPARSE_TO_IDX = {0:0, 1:1, 2:2, 3:4}
REAL_CAM_WEIGHTS = 'rc.09969.h5'
SIM_CAM_WEIGHTS =  'sc.09860.h5'
MODEL_PICTURE_SIZE = (224, 224)

class TLClassifier(object):
    def __init__(self, is_site=False):
        base_path = os.path.dirname(os.path.abspath(__file__))

        sim_camera_weights = os.path.join(base_path, SIM_CAM_WEIGHTS)
        real_camera_weights = os.path.join(base_path, REAL_CAM_WEIGHTS)

        self.model = self.create_model()
        if is_site:
            self.model.load_weights(real_camera_weights)
        else:
            self.model.load_weights(sim_camera_weights)
        
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
            tl = TrafficLight()
            tl.state = color
            return tl
