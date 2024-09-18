#!/usr/bin/python3
import tank_driver
import rospy
import time
import threading
import signal
import sys

from geometry_msgs.msg import Twist, TwistStamped

L = 0.14258 # distance between left and right wheels
TICKSPERDIST = 1111 # should be changed to actual value


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
        self.pub_speed = rospy.Publisher("/speed", TwistStamped, queue_size=100)
        
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

            msg = TwistStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'tank'
            msg.twist.linear.x = (left_speed + right_speed)/2.0
            msg.twist.angular.z = (right_speed - left_speed)/L

            self.pub_speed.publish(msg)
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