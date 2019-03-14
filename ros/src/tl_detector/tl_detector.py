#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
from scipy.spatial import KDTree
import tf
import cv2
import yaml
import math
import numpy as np
from threading import Thread

STATE_COUNT_THRESHOLD = 1
TL_LOOK_AHEAD = 100
TL_LOOK_BEHIND = 2

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoint_tree = None
        self.camera_image = None
        self.lights = []
        self.bridge = CvBridge()
        self.state = TrafficLight.UNKNOWN
        self.lock = False

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        model_weights = self.config['model_weights']
        self.light_classifier = TLClassifier(model_weights)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/image_color', Image, self.image_cb)
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        if not self.waypoint_tree:
            waypoints_2d = [
                    [w.pose.pose.position.x, w.pose.pose.position.y]
                    for w in waypoints.waypoints
                    ]
            self.waypoint_tree = KDTree(waypoints_2d)

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.camera_image = msg
        self.process_traffic_lights()

    def is_stop_tl_state(self, tl_state):
        return tl_state == TrafficLight.RED or tl_state == TrafficLight.YELLOW

    def publish_lights(self, light_wp, state):
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if self.is_stop_tl_state(state) else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1


    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        px = pose.position.x
        py = pose.position.y
        closest_idx = -1
        if self.waypoint_tree is not None:
            closest_idx = self.waypoint_tree.query([px, py], 1)[1]

        return closest_idx

    def get_light_state(self):
        if(not self.camera_image):
            self.prev_light_loc = None
            return TrafficLight.UNKNOWN

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")
        light = self.light_classifier.get_classification(cv_image)
        return light.state

    def get_stop_waypoint(self, car_pos):
        stop_line_positions = self.config['stop_line_positions']
        for sl_x, sl_y in stop_line_positions:
            stop_pose = Pose()
            stop_pose.position.x = sl_x
            stop_pose.position.y = sl_y
            # get the wp closest to each light_position
            stop_wp = self.get_closest_waypoint(stop_pose)

            # return the stopine's wp index
            # if the stopline is ahead the car within the TL_LOOK_AHEAD wps range,
            # or it is behind the car withing the TL_LOOK_BEHIND wps range.
            # the stopline can be behind if the car could not stop and crossed the stopline.
            # in this case the cars must also stop.
            if car_pos - TL_LOOK_BEHIND <= stop_wp and stop_wp <= car_pos + TL_LOOK_AHEAD:
                return stop_wp
        return None

    def is_tl_visible(self, stop_wp, car_pos):
        dist_to_light = 10000
        if stop_wp and car_pos:
            dist_to_light = abs(car_pos - stop_wp)
        return dist_to_light < TL_LOOK_AHEAD

    def process_traffic_lights(self):
        """Finds closest visible traffic light,
            if one exists, and determines its
            location and color
        """
        car_pos = None
        if self.pose:
            car_pos = self.get_closest_waypoint(self.pose.pose)
            stop_wp = self.get_stop_waypoint(car_pos)
            if self.is_tl_visible(stop_wp, car_pos):
                thread = Thread(
                        target = self.predict_and_publish,
                        args=[stop_wp],
                        )
                thread.start()


    def predict_and_publish(self, wp):
        if self.lock:
            return
        self.lock=True
        state = self.get_light_state()
        self.publish_lights(wp, state)
        self.lock=False


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
