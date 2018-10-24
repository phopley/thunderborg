#!/usr/bin/env python
import sys
import rospy
import thunderborg_lib
import pid_lib
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

        # Configure the PIDs
        self.pid1__ = pid_lib.PID(0.8, 2.5, 0.0) # TODO P, I & D values to become parameters in the parameter server
        self.pid1__.SetPoint = 0.0
        self.pid1__.setSampleTime(0.25)

        self.pid2__ = pid_lib.PID(0.8, 2.5, 0.0) # TODO P, I & D values to become parameters in the parameter server
        self.pid2__.SetPoint = 0.0
        self.pid2__.setSampleTime(0.25)

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
	
        # TODO This next bit is a start but we need the odometer feedback and a PID to get the output speed matching the demand speed
        # (OK for teleop)
        #self.__thunderborg.SetMotor1(speed_wish_right/SPEED_RATIO)
        #self.__thunderborg.SetMotor2(speed_wish_left/SPEED_RATIO)
        
        # Update any change to setpoint
        # TODO 1.27 should come from parameter server it is the actual top loaded speed. We limit our speed to 1m/s
        self.pid1__.SetPoint = speed_wish_right/SPEED_RATIO
        self.pid2__.SetPoint = speed_wish_left/SPEED_RATIO

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

    # Update the PIDs with the last feedback values and set the motor speeds
    def RunPID(self):
        # Feedback does contain a sign so
        # set the sign based on the direction that we last commanded the motors
#        if(self.motor1Speed__ < 0):
#            self.feedback1__ = -(self.feedback1__)            
#        if(self.motor2Speed__ < 0):
#            self.feedback2__ = -(self.feedback2__)

        # Update the PIDS
        self.pid1__.update(self.feedback1__/SPEED_RATIO)
        self.pid2__.update(self.feedback2__/SPEED_RATIO)

        # Use the current PID outputs to adjust the motor values
        self.motor1Speed__ = self.pid1__.output
        self.motor2Speed__ = self.pid2__.output

        self.__thunderborg.SetMotor1(self.motor1Speed__)
        self.__thunderborg.SetMotor2(self.motor2Speed__)
        
        pid_state = Vector3()
        pid_state.x = self.pid1__.PTerm
        pid_state.y = self.pid1__.ITerm
        pid_state.z = self.pid1__.DTerm
        self.__pid_pub.publish(pid_state)
        
        motor_state = Vector3()
        motor_state.x = self.pid1__.SetPoint
        motor_state.y = self.feedback1__/SPEED_RATIO
        motor_state.z = self.pid1__.output
        self.__diag_pub.publish(motor_state)        

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
            
        # Run the pid
        tbn.RunPID()
            
        rate.sleep()
               

if __name__ == '__main__':
    main(sys.argv)
