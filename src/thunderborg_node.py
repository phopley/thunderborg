#!/usr/bin/env python
import sys
import rospy
import thunderborg_lib
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist

class ThunderBorgNode:
    def __init__(self):
        self.__thunderborg = thunderborg_lib.ThunderBorg()  # create the thunderborg object
        self.__thunderborg.Init()
        if not self.__thunderborg.foundChip:
            rospy.logdebug("ThunderBorg board not found")
        else:
            # Setup board to turn off motors if we don't send a message every 1/4 second          
            self.__thunderborg.SetCommsFailsafe(True)
            
        # Subscribe to topics
        self.__vel_sub = rospy.Subscriber("cmd_vel",Twist, self.VelCallback)
                
        # Publish topics
        self.__status_pub = rospy.Publisher("main_battery_status", BatteryState, queue_size=1)

    # Callback for cmd_vel message
    def VelCallback(self, msg):
        WHEEL_DIST = 0.230 # TODO this will become a parameter in the parameter server        
        # Calculate the requested speed of each wheel
        speed_wish_right = ((msg.angular.z * WHEEL_DIST) / 2) + msg.linear.x
        speed_wish_left = (msg.linear.x * 2) - speed_wish_right
                
        # TODO This next bit is a start but we need the odometer feedback and a PID to get the output speed matching the demand speed
        # wish_speed/max speed (1.0 m/s) gives the Thunderborg motor value (OK for teleop)
        self.__thunderborg.SetMotor1(speed_wish_right/1.0)
        self.__thunderborg.SetMotor2(speed_wish_left/1.0)
        
    
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
    
    rate = rospy.Rate(10) 
    
    status_time = rospy.Time.now()
    
    while not rospy.is_shutdown():
        if rospy.Time.now() > status_time:
            tbn.PublishStatus()
            status_time = rospy.Time.now() + rospy.Duration(1)
            
        rate.sleep()
               

if __name__ == '__main__':
    main(sys.argv)
