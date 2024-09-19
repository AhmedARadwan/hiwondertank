#!/usr/bin/python3
import tank_driver
import rospy
import time
import threading
import signal
import sys

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

        self.l_start, self.r_start = self.robot.read_encoder_ticks()
        
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

            left_distance = (left_ticks - self.l_start)/TICKSPERMETER
            right_distance = (right_ticks - self.r_start)/TICKSPERMETER

            odom_msg = Odometry()
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "tank"

            pose_msg = Pose()
            pose_msg.position.x = (left_distance + right_distance)/2.0
            pose_msg.position.y = 0

            yaw = (left_distance - right_distance)/L

            quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
            pose_msg.orientation.x = yaw*57.2958
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