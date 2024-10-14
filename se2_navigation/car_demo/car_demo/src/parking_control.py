#!/usr/bin/env python3

import rospy, os
from math import cos, sin, pi, sqrt, pow, atan2
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from erp42_msgs.msg import DriveCmd, ModeCmd
import numpy as np
import csv
import tf
from tf.transformations import euler_from_quaternion

class parking_control:
    def __init__(self):
        rospy.init_node('parking_control', anonymous=True)
        rospy.Subscriber("/oooooooooooooooooo", Odometry, self.odom_callback)

        self.drive_pub = rospy.Publisher('drive', DriveCmd, queue_size=1)
        self.mode_pub = rospy.Publisher('mode', ModeCmd, queue_size=10)

        self.is_odom = False
        self.current_position = Point()
        self.vehicle_yaw = 0.0

        self.drive_cmd = DriveCmd()
        self.mode_cmd = ModeCmd()

        self.steering = 0.0
        self.prev_gear = 0x00
        self.lfd = 5  # Look Forward Distance

        self.path_data = self.read_path_csv("/home/yeong/my_ws/catkin_ws/path_data.csv")
        self.current_segment_idx = 0

        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            if self.is_odom:
                self.pure_pursuit_control()
            rate.sleep()

    def read_path_csv(self, file_path):
        path_data = []
        with open(file_path, 'r') as csvfile:
            csvreader = csv.DictReader(csvfile)
            for row in csvreader:
                direction = row["Direction"]
                point = Point(float(row["X"]), float(row["Y"]), float(row["Z"]))
                path_data.append({"direction": direction, "point": point})
        return path_data

    def pure_pursuit_control(self):

        vehicle_position = self.current_position
        translation = [vehicle_position.x, vehicle_position.y]

        t = np.array([
            [cos(self.vehicle_yaw), -sin(self.vehicle_yaw), translation[0]],
            [sin(self.vehicle_yaw), cos(self.vehicle_yaw), translation[1]],
            [0, 0, 1]
        ])

        det_t = np.array([
            [t[0][0], t[1][0], -(t[0][0] * translation[0] + t[1][0] * translation[1])],
            [t[0][1], t[1][1], -(t[0][1] * translation[0] + t[1][1] * translation[1])],
            [0, 0, 1]
        ])

        for idx, segment in enumerate(self.path_data[self.current_segment_idx:]):
            local_path_point = det_t.dot([segment["point"].x, segment["point"].y, 1])

            if local_path_point[0] > 0:
                dis = sqrt(pow(local_path_point[0], 2) + pow(local_path_point[1], 2))
                if dis >= self.lfd:
                    self.steering = atan2(local_path_point[1], local_path_point[0])
                    self.update_control(segment["direction"])
                    break

    def update_control(self, direction):

        if direction == "FORWARD":
            gear = 0x00
        elif direction == "BACKWARDS":
            gear = 0x02
        else:
            gear = self.prev_gear

        if gear != self.prev_gear:
            self.mode_cmd.EStop = 0x01
            rospy.sleep(1)
            self.mode_cmd.EStop = 0x00


        self.drive_cmd.KPH = 4 
        self.drive_cmd.Deg = int(self.steering * 180 / pi) 
        self.drive_cmd.brake = 0

        self.mode_cmd.MorA = 0x01
        self.mode_cmd.Gear = gear

        self.drive_pub.publish(self.drive_cmd)
        self.mode_pub.publish(self.mode_cmd)

        self.prev_gear = gear

        os.system('clear')
        print("-------------------------------------")
        print(" Gear: ", "FORWARD" if gear == 0x00 else "BACKWARDS")
        print(" Steering (deg): ", self.drive_cmd.Deg)
        print(" Speed (kph): ", self.drive_cmd.KPH)
        print("-------------------------------------")

    def odom_callback(self, msg):
        self.is_odom = True
        odom_quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        _, _, self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y

if __name__ == '__main__':
    try:
        parking_control()
    except rospy.ROSInterruptException:
        pass
