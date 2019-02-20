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

STATE_COUNT_THRESHOLD = 3
TL_LOOK_AHEAD =100 

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.waypoints_2d = None
        self.camera_image = None
        self.lights = []
        self.has_image = False
        self.waypoint_tree = None
        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.state = TrafficLight.UNKNOWN
        self.lock = False

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.listener = tf.TransformListener()

        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

        # TODO: delete when start using camera images
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

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        # Publish upcoming red lights at camera frequency.
        #self.update_traffic_lights()

    def update_traffic_lights(self):
        '''
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        light_wp, state = self.process_traffic_lights()
        #self.publish_lights(light_wp, state)

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
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        #return light.state
        '''
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)
	'''
        
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        light = self.light_classifier.get_classification(cv_image)
        return light.state

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        closest_light = None
        closest_light_stop_wp = None
        dist_to_light = 10000  # initialize to high value
        first = True
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)
        else:
            return -1, TrafficLight.UNKNOWN

        for i, light in enumerate(self.lights):
            light_stop_pose = Pose()
            light_stop_pose.position.x = stop_line_positions[i][0]
            light_stop_pose.position.y = stop_line_positions[i][1]
            # get the wp closest to each light_position
            light_stop_wp = self.get_closest_waypoint(light_stop_pose)

            if light_stop_wp >= car_position:  # if waypoint is  close to the traffic light and ahead of the car
                if first:  # check for the first light
                    closest_light_stop_wp = light_stop_wp
                    #closest_light = light
                    first = False
                elif light_stop_wp < closest_light_stop_wp:
                    closest_light_stop_wp = light_stop_wp
                    #closest_light = light

        if closest_light_stop_wp is not None:
            dist_to_light = abs(car_position - closest_light_stop_wp)
            # we check the status of the traffic light if it's within TL_LOOK_AHEAD waypoints
            if dist_to_light < TL_LOOK_AHEAD:
                if(not self.lock):
                    thread = Thread(
                            target = self.predict_and_publish, 
                            args=[closest_light_stop_wp],
                            )
                    thread.start()
                #return closest_light_stop_wp, state

        return -1, TrafficLight.UNKNOWN

    def predict_and_publish(self, wp):
        self.lock=True
        state = self.get_light_state()
        self.publish_lights(wp, state)
        self.lock=False
            

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
