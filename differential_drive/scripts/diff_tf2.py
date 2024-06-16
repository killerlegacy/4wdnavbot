#!/usr/bin/env python

import rospy
from math import sin, cos, pi
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Int32

class DiffTf:
    def __init__(self):
        rospy.init_node("diff_tf")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        
        # Parameters
        self.rate = rospy.get_param('~rate', 10.0)  # Rate at which to publish the transform
        self.ticks_meter = float(rospy.get_param('~ticks_meter', 24651))  # Wheel encoder ticks per meter of travel
        self.base_width = float(rospy.get_param('~base_width', 0.245))  # Wheel base width in meters
        
        self.base_frame_id = rospy.get_param('~base_frame_id', 'base_link')  # Base frame of the robot
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')  # Odometry reference frame
        
        self.encoder_min = rospy.get_param('~encoder_min', -32768)
        self.encoder_max = rospy.get_param('~encoder_max', 32768)
        self.encoder_low_wrap = rospy.get_param('~wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min)
        self.encoder_high_wrap = rospy.get_param('~wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min)
 
        # Internal data
        self.enc_left = None  # Wheel encoder readings
        self.enc_right = None
        self.left = 0  # Actual values coming back from robot
        self.right = 0
        self.lmult = 0
        self.rmult = 0
        self.prev_lencoder = 0
        self.prev_rencoder = 0
        self.x = 0  # Position in xy plane 
        self.y = 0
        self.th = 0  # Orientation
        self.dx = 0  # Speeds in x/rotation
        self.dr = 0
        self.then = rospy.Time.now()
        
        # Subscriptions
        rospy.Subscriber("left_ticks", Int32, self.lwheelCallback)
        rospy.Subscriber("right_ticks", Int32, self.rwheelCallback)
        self.odomPub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.odomBroadcaster = TransformBroadcaster()

    def spin(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()

    def update(self):
        now = rospy.Time.now()
        elapsed = (now - self.then).to_sec()
        self.then = now
        
        # Calculate odometry
        if self.enc_left is not None:
            d_left = (self.left - self.enc_left) / self.ticks_meter
            d_right = (self.right - self.enc_right) / self.ticks_meter
        else:
            d_left = 0
            d_right = 0
        
        self.enc_left = self.left
        self.enc_right = self.right
        
        # Distance traveled is the average of the two wheels 
        d = (d_left + d_right) / 2
        # This approximation works (in radians) for small angles
        th = (d_right - d_left) / self.base_width
        # Calculate velocities
        self.dx = d / elapsed if elapsed > 0 else 0
        self.dr = th / elapsed if elapsed > 0 else 0
        
        if d != 0:
            # Calculate distance traveled in x and y
            x = cos(th) * d
            y = -sin(th) * d
            # Calculate the final position of the robot
            self.x += cos(self.th) * x - sin(self.th) * y
            self.y += sin(self.th) * x + cos(self.th) * y
        if th != 0:
            self.th += th
        
        # Publish the odom information
        quaternion = Quaternion()
        quaternion.z = sin(self.th / 2)
        quaternion.w = cos(self.th / 2)

        self.odomBroadcaster.sendTransform(
            (self.x, self.y, 0),
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
            rospy.Time.now(),
            self.base_frame_id,
            self.odom_frame_id
        )

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation = quaternion
        odom.child_frame_id = self.base_frame_id
        odom.twist.twist.linear.x = self.dx
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = self.dr
        self.odomPub.publish(odom)

    def lwheelCallback(self, msg):
        enc = msg.data
        if enc < self.encoder_low_wrap and self.prev_lencoder > self.encoder_high_wrap:
            self.lmult += 1
        if enc > self.encoder_high_wrap and self.prev_lencoder < self.encoder_low_wrap:
            self.lmult -= 1
        self.left = 1.0 * (enc + self.lmult * (self.encoder_max - self.encoder_min))
        self.prev_lencoder = enc

    def rwheelCallback(self, msg):
        enc = msg.data
        if enc < self.encoder_low_wrap and self.prev_rencoder > self.encoder_high_wrap:
            self.rmult += 1
        if enc > self.encoder_high_wrap and self.prev_rencoder < self.encoder_low_wrap:
            self.rmult -= 1
        self.right = 1.0 * (enc + self.rmult * (self.encoder_max - self.encoder_min))
        self.prev_rencoder = enc

if __name__ == '__main__':
    try:
        diffTf = DiffTf()
        diffTf.spin()
    except rospy.ROSInterruptException:
        pass
