#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import TrafficLightArray
from sensor_msgs.msg import Image
import numpy as np
import math
from cv_bridge import CvBridge, CvBridgeError
import time
import cv2
from scipy.spatial import KDTree
import yaml
import shutil
import os


class TrainingSetGenerator(object):
    def __init__(self):
        rospy.init_node('trainingset_generator')

        self.pose = None
        self.tl_tree = None
        self.tl_2d = None
        self.lights = None
        self.image_dir = self.get_image_dir()

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)

        self.bridge = CvBridge()
        rospy.Subscriber('/image_color', Image, self.image_cb)
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        
        self.create_dirs()

        self.loop()

    def loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
            
    def get_image_dir(self):
        if os.environ.get('IMAGE_DIR'):
            return os.environ['IMAGE_DIR']
        return '/capstone/ros/camera_img'

    def create_dirs(self):
        if os.path.exists(self.image_dir):
            shutil.rmtree(self.image_dir)
        for tl_state in [0, 1, 2, 4]:
            directory = "{}/{}".format(self.image_dir, tl_state)
            os.makedirs(directory)    


    def millis(self):
        return int(round(time.time() * 1000))

    def traffic_cb(self, msg):
        self.lights = msg.lights
        if not self.tl_tree:
            self.tl_2d = [[w.pose.pose.position.x, w.pose.pose.position.y] for w in self.lights]
            self.tl_tree = KDTree(self.tl_2d)


    def process_traffic_lights(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y

        closest_idx = self.tl_tree.query([x, y], 1)[1]
        prev_idx = (closest_idx -1) % len(self.tl_2d)

        nearest_coord = self.tl_2d[closest_idx]
        prev_coord = self.tl_2d[prev_idx]

        nearest_vect = np.array(nearest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x,y])

        product = np.dot(nearest_vect - prev_vect, pos_vect - nearest_vect)

        if product  > -130000 and product < -10000 :
            tl = self.lights[closest_idx]
            return (product, tl.state, closest_idx)
        elif product > 0:
            return (product, 4, 'NO_TL')
        return None

    def pose_cb(self, msg):
        self.pose = msg

    def image_cb(self, data):

        if not self.tl_tree:
            return

        tl_info = self.process_traffic_lights()
        if tl_info:
            (product, state, idx) = tl_info
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                image_name = "{}-{}-{}.png".format(idx, product, self.millis())
                image_dir = "{}/{}".format(self.image_dir, state)
                cv2.imwrite(
                        "{}/{}".format(image_dir, image_name), 
                        cv_image,
                        )
            except CvBridgeError as e:
                print(e)

    def get_closest_waypoint(self, x, y):
        closest_idx = self.tl_tree.query([x, y], 1)[1]
        return closest_idx


if __name__ == '__main__':
    try:
        TrainingSetGenerator()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start training set generator node.')
