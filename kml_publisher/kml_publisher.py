# Copyright 2023 Scott Horton
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import numpy as np
import pathlib
import time

import rclpy

from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

class KmlPublisher(Node):

    def __init__(self):
        super().__init__('kml_publisher')

        kml_filepath_desc = ParameterDescriptor(description='Full path to the KML file to create')
        self.declare_parameter('kml_filepath', 'robot_pose.kml', kml_filepath_desc)

        def_robot_icon_path = str(pathlib.Path(__file__).parent.absolute()) + '/icons/default_robot.png'
        robot_icon_filepath_desc = ParameterDescriptor(description='Full path to the icon to represent to robot in the KML file')
        self.declare_parameter('robot_icon_filepath', def_robot_icon_path, robot_icon_filepath_desc)

        self.sub_odom = self.create_subscription(
            Odometry,
            '/odometry/global',
            self.odom_cb,
            10)
        self.sub_odom  # prevent unused variable warning

        self.sub_gps = self.create_subscription(
            NavSatFix,
            '/gps/filtered',
            self.gps_cb,
            10)
        self.sub_gps  # prevent unused variable warning
        
        self.last_yaw = -1
        self.last_pub_time = 0

    def odom_cb(self, msg):
        r, p, y = self.euler_from_quaternion2(msg.pose.pose.orientation.x,
                                             msg.pose.pose.orientation.y,
                                             msg.pose.pose.orientation.z,
                                             msg.pose.pose.orientation.w)
        yaw = y*180.0/math.pi
        yaw_kml = (yaw - 90.0) * -1
        if yaw_kml < 0.0:
            yaw_kml += 360.0

        self.last_yaw = yaw_kml
        #self.get_logger().info("odom, x,y: %f, %f, yaw, yaw_kml: %f %f" % (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw, yaw_kml))

    def gps_cb(self, msg):
        if self.last_yaw != -1:
            self.get_logger().info("gps, lat,long: %f, %f, yaw_kml %f" % (msg.latitude, msg.longitude, self.last_yaw))
            
            self.create_kml_file(msg.latitude, msg.longitude, self.last_yaw)


    def create_kml_file(self, latitude, longitude, yaw):

        time_now = time.monotonic()
        if time_now - self.last_pub_time < 1:
            return
        self.last_pub_time = time_now

        kml_template_path = str(pathlib.Path(__file__).parent.absolute()) + '/kml/template.kml'
        icon_path = self.get_parameter('robot_icon_filepath').get_parameter_value().string_value
        kml_filepath = self.get_parameter('kml_filepath').get_parameter_value().string_value

        with open(kml_template_path) as f:
            template = f.read()

        # Google Earth seems to ignore heading if equal to 0
        if yaw == 0.0:
            yaw = 0.1

        template = template.replace('REPLACE_ICON_PATH', icon_path)
        template = template.replace('REPLACE_HEADING', str(yaw))
        template = template.replace('REPLACE_LONG', str(longitude))
        template = template.replace('REPLACE_LAT', str(latitude))

        self.get_logger().debug("KML: %s" % template)

        f = open(kml_filepath, "w")
        f.write(template)
        f.close()
        self.get_logger().info("Published kml to %s" % kml_filepath)

    # https://gist.github.com/salmagro/2e698ad4fbf9dae40244769c5ab74434
    def euler_from_quaternion2(self, x, y, z, w):
        """
        Converts quaternion to euler roll, pitch, yaw
        """

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)

    kml_publisher = KmlPublisher()

    rclpy.spin(kml_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imu_filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()