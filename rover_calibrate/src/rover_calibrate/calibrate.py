#! /usr/bin/python
#***********************************************************
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

# Author: Wim Meeussen

from __future__ import with_statement

import roslib; roslib.load_manifest('turtlebot_calibration')
import yaml
import PyKDL
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from turtlebot_calibration.msg import ScanAngle
from math import *
import threading


def quat_to_angle(quat):
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    return rot.GetRPY()[2]
        
def normalize_angle(angle):
    res = angle
    while res > pi:
        res -= 2.0*pi
    while res < -pi:
        res += 2.0*pi
    return res


class CalibrateRobot:
    def __init__(self):
        self.lock = threading.Lock()
        self.sub_imu  = rospy.Subscriber('imu', Imu, self.imu_cb)
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.odom_cb)
        self.sub_scan = rospy.Subscriber('scan_angle', ScanAngle, self.scan_cb)
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist)
        self.imu_time = rospy.Time()
        self.odom_time = rospy.Time()
        self.scan_time = rospy.Time()
        
        # params
        self.inital_wall_angle = rospy.get_param("inital_wall_angle", 0.1)
        self.imu_calibrate_time = rospy.get_param("imu_calibrate_time", 10.0)
        


    def calibrate(self, speed, imu_drift=0):
        # rotate 360 degrees
        (imu_start_angle, odom_start_angle, scan_start_angle, 
         imu_start_time, odom_start_time, scan_start_time) = self.sync_timestamps()
        last_angle = odom_start_angle
        turn_angle = 0
        while turn_angle < 2*pi:
            if rospy.is_shutdown():
                return
            cmd = Twist()
            cmd.angular.z = speed
            self.cmd_pub.publish(cmd)
            rospy.sleep(0.1)
            with self.lock:
                delta_angle = normalize_angle(self.odom_angle - last_angle)
            turn_angle += delta_angle
            last_angle = self.odom_angle
        self.cmd_pub.publish(Twist())

        (imu_end_angle, odom_end_angle, scan_end_angle,
         imu_end_time, odom_end_time, scan_end_time) = self.sync_timestamps()
        imu_delta = 2*pi + normalize_angle(imu_end_angle - imu_start_angle) - imu_drift*(imu_end_time - imu_start_time).to_sec()
        odom_delta = 2*pi + normalize_angle(odom_end_angle - odom_start_angle)
        scan_delta = 2*pi + normalize_angle(scan_end_angle - scan_start_angle)
        rospy.loginfo('Imu error: %f percent'%(100.0*((imu_delta/scan_delta)-1.0)))
        rospy.loginfo('Odom error: %f percent'%(100.0*((odom_delta/scan_delta)-1.0)))
        return (imu_delta/scan_delta, odom_delta/scan_delta)





    def imu_drift(self):
        # estimate imu drift
        rospy.loginfo('Estimating imu drift')
        (imu_start_angle, odom_start_angle, scan_start_angle, 
         imu_start_time, odom_start_time, scan_start_time) = self.sync_timestamps()
        rospy.sleep(self.imu_calibrate_time)
        (imu_end_angle, odom_end_angle, scan_end_angle,
         imu_end_time, odom_end_time, scan_end_time) = self.sync_timestamps()
        imu_drift = normalize_angle(imu_end_angle - imu_start_angle) / ((imu_end_time - imu_start_time).to_sec())
        rospy.loginfo(' ... imu drift is %f degrees per second'%(imu_drift*180.0/pi))
        return imu_drift


    def align(self):
        rospy.loginfo("Aligning base with wall")
        with self.lock:
            angle = self.scan_angle
        cmd = Twist()

        while angle < -self.inital_wall_angle or angle > self.inital_wall_angle:
            if rospy.is_shutdown():
                exit(0)
            if angle > 0:
                cmd.angular.z = -0.3
            else:
                cmd.angular.z = 0.3
            self.cmd_pub.publish(cmd)
            rospy.sleep(0.05)
            with self.lock:
                angle = self.scan_angle





    def sync_timestamps(self, start_time=None):
        if not start_time:
            start_time = rospy.Time.now() + rospy.Duration(0.5)
        while not rospy.is_shutdown():
            rospy.sleep(0.3)
            with self.lock:
                if self.imu_time < start_time:
                    rospy.loginfo("Still waiting for imu")
                elif self.odom_time < start_time:
                    rospy.loginfo("Still waiting for odom")
                elif self.scan_time < start_time:
                    rospy.loginfo("Still waiting for scan")
                else:
                    return (self.imu_angle, self.odom_angle, self.scan_angle,
                            self.imu_time, self.odom_time, self.scan_time)
        exit(0)
        

    def imu_cb(self, msg):
        with self.lock:
            angle = quat_to_angle(msg.orientation)
            self.imu_angle = angle
            self.imu_time = msg.header.stamp

    def odom_cb(self, msg):
        with self.lock:
            angle = quat_to_angle(msg.pose.pose.orientation)
            self.odom_angle = angle
            self.odom_time = msg.header.stamp

    def scan_cb(self, msg):
        with self.lock:
            angle = msg.scan_angle
            self.scan_angle = angle
            self.scan_time = msg.header.stamp



def main():
    rospy.init_node('scan_to_angle')
    robot = CalibrateRobot()
    
    imu_drift = robot.imu_drift()
    imu_corr = []
    odom_corr = []
    for speed in (0.7, 1.0, 1.3, 1.5, 2.0):
        robot.align()
        (imu, odom) = robot.calibrate(speed, imu_drift)
        imu_corr.append(imu)
        odom_corr.append(odom)

    imu_res = 1.0/(sum(imu_corr)/len(imu_corr))
    odom_res = 1.0/(sum(odom_corr)/len(odom_corr))
    rospy.loginfo("Multiply the 'turtlebot_node/gyro_scale_correction' parameter with %f"%imu_res)
    rospy.loginfo("Multiply the 'turtlebot_node/odom_angular_scale_correction' parameter with %f"%odom_res)
                  

    print imu_res
    print odom_res


if __name__ == '__main__':
    main()
