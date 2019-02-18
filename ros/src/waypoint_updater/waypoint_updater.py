#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
import numpy as np
import math
from std_msgs.msg import Int32

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number
MAX_DECEL = 0.5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
	rospy.Subscriber('/traffic_waypoint',Int32, self.traffic_cb)
	
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
	self.stopline_wp_ind = -1

        self.loop()

    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.waypoint_tree:
                # Get nearest waypoint
                #closest_waypoint_idx = self.get_nearest_waypoint_idx()
                self.publish_waypoints()
            rate.sleep()

    def publish_waypoints(self):
	final_lane = self.generate_lane()
	self.final_waypoints_pub.publish(final_lane)
	'''
        lane = Lane()
        lane.header = self.base_waypoints.header
        lane.waypoints = self.base_waypoints.waypoints[idx : idx + LOOKAHEAD_WPS]
        self.final_waypoints_pub.publish(lane)
	'''

    def generate_lane(self):
	#print("Making lane")
	lane = Lane()

	closest_idx = self.get_nearest_waypoint_idx()
	farthest_idx = closest_idx + LOOKAHEAD_WPS
	base_waypoints_1 = self.base_waypoints.waypoints[closest_idx:farthest_idx]
	
	if self.stopline_wp_ind == -1 or (self.stopline_wp_ind >= farthest_idx):
	    print("Light too far\nlight idx={}, far idx={}".format(self.stopline_wp_ind, farthest_idx))
	    lane.waypoints = base_waypoints_1
	else:
	    print("Light is close")
	    lane.waypoints = self.decelerate_waypoints(base_waypoints_1, closest_idx)

	return lane

    def decelerate_waypoints(self, waypoints, closest_idx):
	temp = []
	for i, wp in enumerate(waypoints):
	    p = Waypoint()
	    p.pose = wp.pose

	    stop_idx = max(self.stopline_wp_ind - closest_idx - 2, 0)
	    dist = self.distance(waypoints, i, stop_idx)
            vel = math.sqrt(2 * MAX_DECEL * dist)
	    if vel < 1:
		vel = 0
	    p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
	    temp.append(p)
	return temp

    

    def get_nearest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        #Check if nearest waypoint is ahead or behind the vehicle
        nearest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]

        #Equation for hyperlane through closest coords
        nearest_vect = np.array(nearest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x,y])

        product = np.dot(nearest_vect - prev_vect, pos_vect - nearest_vect)

        if product  > 0:
            closest_idx = (closest_idx+1) % len(self.waypoints_2d)
        return closest_idx

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[w.pose.pose.position.x, w.pose.pose.position.y] for w in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
	self.stopline_wp_ind = msg.data    
	print("recieved waypoit = {}".format(self.stopline_wp_ind))
	  

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
