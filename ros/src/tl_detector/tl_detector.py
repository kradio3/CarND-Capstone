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

STATE_COUNT_THRESHOLD = 2
TL_LOOK_AHEAD = 100

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoint_tree = None
        self.camera_image = None
        self.lights = []
        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.state = TrafficLight.UNKNOWN
        self.lock = False

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg
        self.process_traffic_lights()

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
        closest_stop_wp = None
        first = True
        for sl_x, sl_y in stop_line_positions:
            stop_pose = Pose()
            stop_pose.position.x = sl_x
            stop_pose.position.y = sl_y
            # get the wp closest to each light_position
            stop_wp = self.get_closest_waypoint(stop_pose)

            # if waypoint is  close to the traffic light and ahead of the car
            if stop_wp >= car_pos:
                # check for the first light
                if first:
                    closest_stop_wp = stop_wp
                    first = False
                elif stop_wp < closest_stop_wp:
                    closest_stop_wp = stop_wp
        return closest_stop_wp

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
        closest_light = None
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
