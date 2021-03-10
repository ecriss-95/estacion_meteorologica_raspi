#!/usr/bin/env python

import os
import sys
import json
import numpy as np
import time
import atexit
import rospy
import tf
from robot_controller import ArduinoSerial
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import Float64, String
from tf.transformations import quaternion_from_euler

try:
    from control import SonoffController
    SONOFF_LOADED = True
except:
    SONOFF_LOADED = False

SONOFF_LOADED = False

class LuciaInterface(object):
    """
    Establish communication between mobile base and ROS.

    ROS node name:
        lucia_controller

    ROS topics published:
        /odom
        /battery_voltage
        /battery_status
        /bumpers_active
        /e_stop
        /sonar_left
        /sonar_center
        /sonar_right
        /sonar_back
        /dock_current

    ROS topics subscribed:
        /cmd_vel
        /enableMotors
        /stop_motors
        /door
    """

    def __init__(self):
        """
        Initialize ROS node.

        ROS parameters:
            ~port: Microcontroller's serial port.
            ~baud: Baud rate used in serial communication.
            ~timeout: Read timeout in seconds.
            ~debug: Log debug info or not.
            /mobile_base/using_sonoff: Sonoff used or not for cage.
            /mobile_base/door_open_seconds: Seconds opening door.
            /mobile_base/door_close_seconds: Seconds closing door.
        """
        rospy.init_node("lucia_controller")

        self.port = rospy.get_param("~port", "/dev/DueController")
        self.baud = rospy.get_param("~baud", 38400)
        self.timeout = rospy.get_param("~timeout", 1)
        self.debug = rospy.get_param("~debug", 0)
        self.using_sonoff = rospy.get_param('/mobile_base/using_sonoff', 0)
        
        self.door_open_seconds = rospy.get_param('/mobile_base/door_open_seconds', 30)
        self.door_close_seconds = rospy.get_param('/mobile_base/door_close_seconds', 30)

        if self.using_sonoff:
            if not SONOFF_LOADED:
                rospy.logerr("[ROBOT CONTROLLER] Couldn't import SonoffController library. Not using Sonoff")
                self.using_sonoff = 0
            else:
                try:
                    self.sonoff = SonoffController("192.168.1.8")
                    rospy.loginfo("[ROBOT CONTROLLER] Using Sonoff door control")
                except Exception as e:
                    rospy.logerr("Error in Sonoff: " + str(e) + '. Not using Sonoff')
                    self.using_sonoff = 0

        self.period = 0.01
        self.ready = False
        self.timer = None
        self.seq = 0

        self.bumpersOn = False
        self.stopOn = False
        self.odomTF = None
        self.baselinkTF = None
        self.battery_voltage = 48.0

        self.arduino = None

        rospy.loginfo("[ROBOT CONTROLLER] Trying to connect to mobile base (port=%s, baud=%d, timeout=%f)"
                      % (self.port, self.baud, self.timeout))

        try:
            # Create an instance of ArduinoSerial to communicate with arduino via serial port
            self.arduino = ArduinoSerial(self.port, self.baud, self.timeout)

            if self.arduino.open_serial():
                atexit.register(self.kill_arduino)
                self.ready = True
                self.is_stopped = False

                # Define ROS publishers and subscribers for mobile base topics
                self.odomPub = rospy.Publisher('/odom', Odometry, queue_size=5)
                self.battPub = rospy.Publisher('/battery_voltage', Float64, queue_size=5)
                self.battStatusPub = rospy.Publisher('/battery_status', Float64, queue_size=5)
                self.bumperPub = rospy.Publisher('/bumpers_active', Float64, queue_size=5)
                self.stopPub = rospy.Publisher('/e_stop', Float64, queue_size=5)
                self.sonar_leftPub = rospy.Publisher('/sonar_left', Range, queue_size=5)
                self.sonar_centerPub = rospy.Publisher('/sonar_center', Range, queue_size=5)
                self.sonar_rightPub = rospy.Publisher('/sonar_right', Range, queue_size=5)
                self.sonar_backPub = rospy.Publisher('/sonar_back', Range, queue_size=5)
                self.dockCurrPub = rospy.Publisher('/dock_current', Float64, queue_size=5)

                self.cmdvelSub = rospy.Subscriber('cmd_vel', Twist, self._cmd_vel_cb)
                self.enableMotorsSub = rospy.Subscriber('/enableMotors', Float64, self._enable_motors)
                self.stopMotorsSub = rospy.Subscriber('/stopMotors', Float64, self._stop_motors)
                self.doorSub = rospy.Subscriber('/door', String, self._door_actions)

                self.odomTF = tf.TransformBroadcaster()

                # init timer to get readings from arduino
                self.timer = rospy.Timer(rospy.Duration(self.period), self.read_from_arduino)
                rospy.loginfo("[ROBOT CONTROLLER] Connected! Ready to operate")

                self._enable_motors(Float64(1.0))
            else:
                rospy.logerr("[ROBOT CONTROLLER] Unable to open connection to mobile base. Please check controller.")
        except Exception as e:
            rospy.logerr("[ROBOT CONTROLLER] Unable to connect to mobile base: " + str(e))

    def _stop_motors(self, data):
        """
        Stop mobile base.

        Arguments inside data:
            data: 0.0 doesn't stop
                  1.0 stops
                  2.0 bumper used. Move backwards
        """
        try:
            if self.arduino:
                if data.data == 1.0:
                    self.is_stopped = True
                    self.arduino.send_velocity(0.0, 0.0)
                elif data.data == 0.0:
                    self.is_stopped = False
                elif data.data == 2.0:  # bumper
                    self.is_stopped = True
                    self.arduino.send_velocity(-0.05, 0.0)
                    time.sleep(2.0)
                    self.arduino.send_velocity(0.0, 0.0)
        except Exception as e:
            rospy.logerr("[ROBOT CONTROLLER] Error while stopping motors: " + str(e))

    def _enable_motors(self, data):
        """Send command to enable mobile base motors."""
        try:
            if self.arduino:
                rospy.loginfo("[ROBOT CONTROLLER] Enabling motors...")
                self.arduino.send_enabler(True)
                rospy.loginfo("[ROBOT CONTROLLER] Motors ready!")
        except Exception as e:
            rospy.logerr("[ROBOT CONTROLLER] Error while enabling motors: " + str(e))

    def _door_actions(self, data):
        if self.arduino:
            msg = json.loads(data.data)
            try:
                door_action = msg['action']
                door_time = 30

                if 'time' in msg:
                    door_time = msg['time']
                else:
                    if door_action == 'open':
                        door_time = self.door_open_seconds
                    elif door_action == 'close':
                        door_time = self.door_close_seconds

                if not self.using_sonoff:
                    door_output = self.arduino.send_door_action(door_action, door_time)
                else:
                    rospy.loginfo('[ROBOT CONTROLLER] Starting sonoff')
                    if door_action == 'open':
                        self.sonoff.open(door_time)
                        door_output = "[ROBOT CONTROLLER] Opening door for {0} seconds".format(door_time)
                    elif door_action == 'close':
                        self.sonoff.close(door_time)
                        door_output = "[ROBOT CONTROLLER] Closing door for {0} seconds".format(door_time)
                    else:
                        door_output = "[ROBOT CONTROLLER] Invalid command"
                rospy.loginfo(door_output)
            except Exception as e:
                rospy.logerr("Error while trying to move door: " + str(e))

    def read_from_arduino(self, data):
        """Read message receive from microcontroller and publish its info to desired topics."""
        message, dmsg, raw_msg = self.arduino.read_message(debug=True)

        if message:
            if self.debug:
                rospy.loginfo(raw_msg)

            odom = Odometry()
            odom.header.frame_id = 'odom'
            odom.header.seq = self.seq
            odom.header.stamp = rospy.Time.now()

            # Publish sonars measurements
            # Max range set in 1 meter since microcontroller uses a 60ms timeout when waiting pulse
            sonar_left = Range()
            sonar_left.header.frame_id = 'sonar_left'
            sonar_left.header.seq = self.seq
            sonar_left.header.stamp = rospy.Time.now()
            sonar_left.radiation_type = Range.ULTRASOUND
            sonar_left.min_range = 0.03
            sonar_left.max_range = 1.0
            sonar_left.field_of_view = np.pi / 6.0

            sonar_center = Range()
            sonar_center.header.frame_id = 'sonar_center'
            sonar_center.header.seq = self.seq
            sonar_center.header.stamp = rospy.Time.now()
            sonar_center.radiation_type = Range.ULTRASOUND
            sonar_center.min_range = 0.03
            sonar_center.max_range = 1.0
            sonar_center.field_of_view = np.pi / 6.0

            sonar_right = Range()
            sonar_right.header.frame_id = 'sonar_right'
            sonar_right.header.seq = self.seq
            sonar_right.header.stamp = rospy.Time.now()
            sonar_right.radiation_type = Range.ULTRASOUND
            sonar_right.min_range = 0.03
            sonar_right.max_range = 1.0
            sonar_right.field_of_view = np.pi / 6.0

            sonar_back = Range()
            sonar_back.header.frame_id = 'sonar_back'
            sonar_back.header.seq = self.seq
            sonar_back.header.stamp = rospy.Time.now()
            sonar_back.radiation_type = Range.ULTRASOUND
            sonar_back.min_range = 0.03
            sonar_back.max_range = 1.0
            sonar_back.field_of_view = np.pi / 6.0

            self.seq += 1

            odom.pose.pose.position.x = dmsg['location'][0]
            odom.pose.pose.position.y = dmsg['location'][1]

            q1, q2, q3, q4 = quaternion_from_euler(0, 0, dmsg['location'][2])

            odom.pose.pose.orientation.x = q1
            odom.pose.pose.orientation.y = q2
            odom.pose.pose.orientation.z = q3
            odom.pose.pose.orientation.w = q4

            odom.twist.twist.linear.x = dmsg['vel'][0]
            odom.twist.twist.angular.z = dmsg['vel'][1]

            self.odomTF.sendTransform((dmsg['location'][0], dmsg['location'][1], 0),
                                      (q1, q2, q3, q4),
                                      rospy.Time.now(),
                                      "base_link",
                                      "odom")

            self.odomPub.publish(odom)

            self.battery_voltage = 0.975 * self.battery_voltage + 0.025 * float(dmsg['battery_voltage'])
            # self.battery_voltage = float(dmsg['battery_voltage'])

            self.battPub.publish(Float64(self.battery_voltage))
            self.battStatusPub.publish(Float64(dmsg['battery_charge_status']))

            # Sonar izquierdo
            sonar_left.range = dmsg['sonar_lectures'][2]
            self.sonar_leftPub.publish(sonar_left)

            # Sonar central
            sonar_center.range = dmsg['sonar_lectures'][1]
            self.sonar_centerPub.publish(sonar_center)

            # Sonar derecho
            sonar_right.range = dmsg['sonar_lectures'][0]
            self.sonar_rightPub.publish(sonar_right)

            # Sonar trasero
            sonar_back.range = dmsg['sonar_lectures'][3]
            self.sonar_backPub.publish(sonar_back)

            self.dockCurrPub.publish(Float64(dmsg['dock_current']))

            if any(dmsg['bumpers']):
                rospy.loginfo("[ROBOT CONTROLLER] Bumpers state: " + str(dmsg['bumpers']))
                if not self.bumpersOn:
                    self.bumperPub.publish(Float64(1.0))
                    rospy.logwarn("[ROBOT CONTROLLER] Detected bumper crash!!")
                    self.bumpersOn = True
            else:
                self.bumpersOn = False

            if dmsg['e_stop']:
                rospy.logwarn("[ROBOT CONTROLLER] Emergency stop state: " + str(dmsg['e_stop']))
                self.stopPub.publish(Float64(1.0))
                if not self.stopOn:
                    # self.stopPub.publish(Float64(1.0))
                    rospy.logwarn("[ROBOT CONTROLLER] Emergency stop activated!!")
                    self.stopOn = True
            else:
                self.stopPub.publish(Float64(0.0))
                if self.stopOn:
                    # self.stopPub.publish(Float64(0.0))
                    rospy.loginfo("[ROBOT CONTROLLER] Emergency stop deactivated")
                    self.stopOn = False

    def _cmd_vel_cb(self, data):
        #print "is_Stopped = %s" % (self.is_stopped)        
        if not self.is_stopped:
            vel_linear = data.linear.x
            vel_angular = data.angular.z
            #print "vel_linear = %s" % (vel_linear)
            self.arduino.send_velocity(vel_linear, vel_angular)

    def kill_arduino(self):
        """Close serial communication with microcontroller."""
        self.arduino.close_serial()
        rospy.loginfo("[ROBOT CONTROLLER] Exit :: Mobile base disconnected successfully")

if __name__ == "__main__":
    while not rospy.is_shutdown():
        luciaInterface = LuciaInterface()
        rospy.spin()