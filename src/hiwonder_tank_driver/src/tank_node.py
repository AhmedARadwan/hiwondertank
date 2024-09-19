#!/usr/bin/python3
import tank_driver
import rospy
import time
import threading
import signal
import sys
import math

import tf
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry

L = 0.14258 # distance between left and right wheels
TICKSPERMETER = 11500.0 # 11500.0


class TankDriver:
    def __init__(self):
        rospy.init_node('tank_controller', anonymous=True)

        signal.signal(signal.SIGINT, self.cleanup)

        self.robot = None
        while self.robot is None:
            try:
                self.robot = tank_driver.DifferentialDriveMotor()
            except:
                print("Motor Controller Board not Powered / I2C pins not connected.\nTrying again in 1 sec....")
                time.sleep(1)
        print("Connected Successfully!")

        rospy.Subscriber('/tank/cmd_vel', Twist, self.callback, queue_size=100)
        self.pub_odom = rospy.Publisher("/odom", Odometry, queue_size=100)

        self.l_prev_ticks, self.r_prev_ticks = self.robot.read_encoder_ticks()

        self.prev_x = 0
        self.prev_y = 0
        self.prev_yaw = 0
        
        # Start measurement thread
        measurement_interval = 0.1
        self.stop_measurement = False
        self.robot.start_speed_measurement(measurement_interval)
        self.measurement_thread = threading.Thread(target= self.publish_speed, args=(measurement_interval,))
        self.measurement_thread.start()

        rospy.spin()

        
    def publish_speed(self, measurement_interval):
        while not self.stop_measurement:
            left_speed, right_speed = self.robot.get_speeds()
            left_ticks, right_ticks = self.robot.read_encoder_ticks()

            dl = (left_ticks - self.l_prev_ticks)/TICKSPERMETER
            dr = (right_ticks - self.r_prev_ticks)/TICKSPERMETER
            dc = (dr + dl)/2.0

            current_yaw = self.prev_yaw + ((dr - dl)/L)
            current_x = self.prev_x + dc * math.cos(current_yaw)
            current_y = self.prev_y + dc * math.sin(current_yaw)

            odom_msg = Odometry()
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "tank"

            pose_msg = Pose()
            pose_msg.position.x = current_x
            pose_msg.position.y = current_y

            quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, current_yaw)
            pose_msg.orientation.x = current_yaw*57.2958
            # pose_msg.orientation.x = quaternion[0]
            # pose_msg.orientation.y = quaternion[1]
            # pose_msg.orientation.z = quaternion[2]
            # pose_msg.orientation.w = quaternion[3]

            twist_msg = Twist()
            twist_msg.linear.x = (left_speed + right_speed)/2.0
            twist_msg.angular.z = (right_speed - left_speed)/L

            odom_msg.twist.twist = twist_msg
            odom_msg.pose.pose = pose_msg

            self.pub_odom.publish(odom_msg)

            self.prev_yaw = current_yaw
            self.prev_x = current_x
            self.prev_y = current_y
            time.sleep(measurement_interval)

    def callback(self, data):
        print("callback")
        offset = (data.angular.z * (L/2.0))
        left_speed = data.linear.x + offset
        right_speed = data.linear.x - offset
        self.robot.set_wheel_speeds(left_speed, right_speed)

    def cleanup(self, signum, frame):
        print("Stopping Motors!")
        self.robot.set_wheel_speeds(0, 0)
        self.stop_measurement = True
        if self.measurement_thread:
            self.measurement_thread.join()
        sys.exit(0)

if __name__ == '__main__':
    try:
        node = TankDriver()
    except rospy.ROSInterruptException:
        pass