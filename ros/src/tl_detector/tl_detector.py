#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
	# TODO: Remove the logger once the node is stable
        rospy.init_node('tl_detector', log_level=rospy.DEBUG)

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

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
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
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
        if self.waypoints is None:
            return

        min_dist = 10000
        closest_wp_index = 0
	# Store x,y in a variable instead of into the object in the loop 
        px = pose.position.x
        py = pose.position.y
        # check all the waypoints to see which one is the closest to our current position
        for i, wp in enumerate(self.waypoints):
        	dist = math.hypot(wp.pose.pose.position.x-px, wp.pose.pose.position.y-py)
		if (dist < min_dist): #we found a closer wp
                	closest_wp_index = i     # we store the index of the closest waypoint
                	min_dist = dist     # we save the distance of the closest waypoint
	
	# TODO: Remove the logger once the node is stable
	rospy.logdebug("TL  DETECTOR CLOSEST INDEX  %s", closest_wp_index)
        # returns the index of the closest waypoint
        return closest_wp_index

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        closest_light_wp = None
        closest_light_stop_wp = None
        dist_to_light = 10000   #initialize to high value

        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            	car_position = self.get_closest_waypoint(self.pose.pose)
        else:
        	return -1, TrafficLight.UNKNOWN

        for light_stop_position in stop_line_positions:
		light_stop_pose = Pose()
            	light_stop_pose.position.x = light_stop_position[0]
            	light_stop_pose.position.y = light_stop_position[1]
            	light_stop_wp = self.get_closest_waypoint(light_stop_pose)    #get the wp closest to each light_position
            	if light_stop_wp >= car_position :    #if waypoint is  close to the traffic light and ahead of the car
            		if closest_light_stop_wp is None:    #check for the first light 
				closest_light_stop_wp = light_stop_wp
                    		light = light_stop_pose
                	elif light_stop_wp < closest_light_stop_wp:
                    		closest_light_stop_wp = light_stop_wp  
                    		light = light_stop_pose

        if ((car_position is not None) and (closest_light_stop_wp is not None)):
        	dist_to_light = abs(car_position - closest_light_stop_wp)

        if light and dist_to_light < 50: #we check the status of the traffic light if it's within 50 waypoints
            	state = self.get_light_state(light)
	    	rospy.logdebug("Closest light position: %s", closest_light_stop_wp)
            	return closest_light_stop_wp, state
        return -1, TrafficLight.UNKNOWN


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
