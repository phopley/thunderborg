#!/usr/bin/env python
import sys
import rospy
import thunderborg_lib
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3   # Temp message for diagnostics
from tacho_msgs.msg import tacho

RATE = 10
#SPEED_RATIO = 1.47 # Battery value
SPEED_RATIO = 1.32 # Test power supply value

class ThunderBorgNode:
    def __init__(self):
        self.__thunderborg = thunderborg_lib.ThunderBorg()  # create the thunderborg object
        self.__thunderborg.Init()
        if not self.__thunderborg.foundChip:
            rospy.logdebug("ThunderBorg board not found")
        else:
            # Setup board to turn off motors if we don't send a message every 1/4 second          
            self.__thunderborg.SetCommsFailsafe(True)

        # Motor velocity feedback values
        self.feedback1__ = 0.0
        self.feedback2__ = 0.0

        # Subscribe to topics
        self.__vel_sub = rospy.Subscriber("cmd_vel",Twist, self.VelCallback)
        self.__feedback_sub = rospy.Subscriber("tacho", tacho, self.TachoCallback) # Used for testing
                
        # Publish topics
        self.__status_pub = rospy.Publisher("main_battery_status", BatteryState, queue_size=1)
        self.__diag1_pub = rospy.Publisher("motor1_diag", Vector3, queue_size=1)
        self.__diag2_pub = rospy.Publisher("motor2_diag", Vector3, queue_size=1)

    # Callback for cmd_vel message
    def VelCallback(self, msg):
        WHEEL_DIST = 0.230 # TODO this will become a parameter in the parameter server        
        # Calculate the requested speed of each wheel
        speed_wish_right = ((msg.angular.z * WHEEL_DIST) / 2) + msg.linear.x
        speed_wish_left = (msg.linear.x * 2) - speed_wish_right

	motor1_speed = speed_wish_right/SPEED_RATIO
	motor2_speed = speed_wish_left/SPEED_RATIO
        self.__thunderborg.SetMotor1(motor1_speed)
        self.__thunderborg.SetMotor2(motor2_speed)

        motor1_state = Vector3()
        motor1_state.x = speed_wish_right
        motor1_state.y = self.feedback1__
        motor1_state.z = motor1_speed
        motor2_state = Vector3()
        motor2_state.x = speed_wish_left
        motor2_state.y = self.feedback2__
        motor2_state.z = motor2_speed

        self.__diag1_pub.publish(motor1_state)
        self.__diag2_pub.publish(motor2_state)

    # Callback for tacho message
    def TachoCallback(self, msg):
        # Store the feedback values for the next time we publish current velocity
        self.feedback1__ = msg.rwheelVel
        self.feedback2__ = msg.lwheelVel
        
    # Publish the battery status    
    def PublishStatus(self):
        battery_state = BatteryState()
        # Read the battery voltage
        battery_state.voltage = self.__thunderborg.GetBatteryReading()        
        self.__status_pub.publish(battery_state)       
        

def main(args):
    rospy.init_node('thunderborg_node', anonymous=False)
    rospy.loginfo("Thunderborg node started")
    tbn = ThunderBorgNode()
    
    rate = rospy.Rate(RATE) 
    
    status_time = rospy.Time.now()
    
    while not rospy.is_shutdown():
        if rospy.Time.now() > status_time:
            tbn.PublishStatus()
            status_time = rospy.Time.now() + rospy.Duration(1)
            
        rate.sleep()
               

if __name__ == '__main__':
    main(sys.argv)
