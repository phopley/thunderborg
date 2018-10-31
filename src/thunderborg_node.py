#!/usr/bin/env python
import sys
import rospy
import thunderborg_lib
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3   # Temp message for diagnostics
from tacho_msgs.msg import tacho

RATE = 10
SPEED_RATIO = 1.47

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
        
        # Current motor speeds
        self.motor1Speed__ = 0.0
        self.motor2Speed__ = 0.0

        # Subscribe to topics
        self.__vel_sub = rospy.Subscriber("cmd_vel",Twist, self.VelCallback)
        self.__feedback_sub = rospy.Subscriber("tacho", tacho, self.TachoCallback)
                
        # Publish topics
        self.__status_pub = rospy.Publisher("main_battery_status", BatteryState, queue_size=1)
        self.__pid_pub = rospy.Publisher("PID", Vector3, queue_size=1)
        self.__diag_pub = rospy.Publisher("DIAGNOSTICS", Vector3, queue_size=1)

    # Callback for cmd_vel message
    def VelCallback(self, msg):
        WHEEL_DIST = 0.230 # TODO this will become a parameter in the parameter server        
        # Calculate the requested speed of each wheel
        speed_wish_right = ((msg.angular.z * WHEEL_DIST) / 2) + msg.linear.x
        speed_wish_left = (msg.linear.x * 2) - speed_wish_right
	
	    # Add in 1/2 the error
	    motor_speed1 = speed_wish_right/SPEED_RATIO+((self.feedback1__/SPEED_RATIO)/2);
	    motor_speed2 = speed_wish_left/SPEED_RATIO+((self.feedback2__/SPEED_RATIO)/2);
	    
        self.__thunderborg.SetMotor1(motor_speed1)
        self.__thunderborg.SetMotor2(motor_speed2)        

    # Callback for tacho message
    def TachoCallback(self, msg):
        # Store the feedback values for the next time we run the PIDs
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
