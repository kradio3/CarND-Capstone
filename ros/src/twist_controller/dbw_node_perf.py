#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
import math

class DBWNodePerf(object):
    def __init__(self):
        rospy.init_node('dbw_node_perf')

        self.freq = 40 # Hz
        self.num_samples = 120 * self.freq # which will be gathered after two-minute driving
        self.mse_acc = 0
        self.mse_cnt = 0

        self.dbw_enabled = False
        self.current_vel = None
        self.current_pose_xy = None
        self.target_linear_vel = None
        self.target_angular_vel = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None        

        # Initialize subscribers
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_vel_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.current_pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        self.loop()

    def loop(self):
        rate = rospy.Rate(self.freq)
        while not rospy.is_shutdown():
            try:
                self.loop_iter()
                rate.sleep()
            except (rospy.ROSInterruptException, KeyboardInterrupt):
                rospy.logwarn('Interrupted. Shutting down the node')
                pass

        mse = self.mse_acc / self.mse_cnt
        rospy.loginfo('Steering performance based on %d samples = %.16f' % (self.mse_cnt, mse))
        
    def loop_iter(self):
        if not self.dbw_enabled:
            return

        if self.current_pose_xy and self.waypoint_tree:
            closest_waypoint_idx = self.get_nearest_waypoint_idx()
            self.update_error(closest_waypoint_idx)

    def shutdown(self, reason):
        rospy.logwarn("Shutdown the node with the reason: %s" % reason)
        rospy.signal_shutdown(reason)

    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = msg

    def current_vel_cb(self, msg):
        self.current_vel = msg.twist.linear.x

    def current_pose_cb(self, msg):
        self.current_pose_xy = [msg.pose.position.x, msg.pose.position.y]

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[w.pose.pose.position.x, w.pose.pose.position.y] for w in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def get_nearest_waypoint_idx(self):
        x = self.current_pose_xy[0]
        y = self.current_pose_xy[1]
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]
        return closest_idx

    def update_error(self, closest_waypoint_idx):
        if self.mse_cnt >= self.num_samples:
            self.shutdown("mse_cnt exceeds num_samples (%d >= %d)" % (self.mse_cnt, self.num_samples))
            return

        closest_wp = self.base_waypoints.waypoints[closest_waypoint_idx]
        # wp_vel = self.get_waypoint_velocity(closest_wp)
        # err_vel = wp_vel - self.current_vel
        err_pose_x = closest_wp.pose.pose.position.x - self.current_pose_xy[0]
        err_pose_y = closest_wp.pose.pose.position.y - self.current_pose_xy[1]

        self.mse_cnt += 1
        self.mse_acc += err_pose_x * err_pose_x
        self.mse_acc += err_pose_y * err_pose_y

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x        

if __name__ == '__main__':
    DBWNodePerf()
