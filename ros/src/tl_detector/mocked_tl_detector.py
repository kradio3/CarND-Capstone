#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from scipy.spatial import KDTree
import cv2
import yaml
import math
import numpy as np

STATE_COUNT_THRESHOLD = 3
TL_LOOK_AHEAD = 100
TL_LOOK_BEHIND = 15

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.waypoints_2d = None
        self.camera_image = None
        self.lights = []
        self.waypoint_tree = None
        self.state = TrafficLight.UNKNOWN

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg
        self.update_traffic_lights()

    def is_stop_tl_state(self, tl_state):
        return tl_state == TrafficLight.RED or tl_state == TrafficLight.YELLOW

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[w.pose.pose.position.x, w.pose.pose.position.y] for w in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def update_traffic_lights(self):
        '''
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        light_wp, state = self.process_traffic_lights()
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

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        return light.state

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if not self.pose:
            return -1, TrafficLight.UNKNOWN

        stop_line_positions = self.config['stop_line_positions']
        car_position = self.get_closest_waypoint(self.pose.pose)

        for i, light in enumerate(self.lights):
            light_stop_pose = Pose()
            light_stop_pose.position.x = stop_line_positions[i][0]
            light_stop_pose.position.y = stop_line_positions[i][1]
            # get the wp closest to each light_position
            light_stop_wp = self.get_closest_waypoint(light_stop_pose)

            if car_position - TL_LOOK_BEHIND <= light_stop_wp and light_stop_wp <= car_position + TL_LOOK_AHEAD:
                state = self.get_light_state(light)
                return light_stop_wp, state

        return -1, TrafficLight.UNKNOWN


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
