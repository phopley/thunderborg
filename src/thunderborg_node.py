#!/usr/bin/env python
import sys
import rospy
import thunderborg_lib
from simple_pid import PID  # See https://pypi.org/project/simple-pid
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from tacho_msgs.msg import tacho
from dynamic_reconfigure.server import Server
from thunderborg.cfg import ThunderborgConfig

RATE = 10

class ThunderBorgNode:
    def __init__(self):
        # Read values from parameter server
        self.__use_pid = rospy.get_param('/pid/use_pid', False)
        self.__wheel_distance = rospy.get_param('/wheels/distance', 0.23)
        self.__wheel_circumfrence = rospy.get_param('/wheels/circumfrence', 0.34)
        self.__speed_slope = rospy.get_param('/speed/slope', 1.5)
        self.__speed_y_intercept = rospy.get_param('/speed/y_intercept', 0.4)
        self.__inertia = rospy.get_param('/pid/inertia_level', 0.3)
        self.__diag_msgs = rospy.get_param('/speed/motor_diag_msg', False)
        
        if self.__use_pid == True:
            # Configure the PIDs
            self.__pid1 = PID(0.6, 0.7, 0.7, setpoint=0)
            self.__pid1.sample_time = 0.05
            # Note that the PID will only deal in positive values, direction of motor
            # will be set by the code writting to the Thunderborg.
            # Limit the pid range
            self.__pid1.output_limits = (self.__inertia, 1.0)
            #self.__pid1.proportional_on_measurement = True
            
            self.__pid2 = PID(0.6, 0.7, 0.7, setpoint=0)
            self.__pid2.sample_time = 0.05
            self.__pid2.output_limits = (self.__inertia, 1.0)
            #self.__pid2.proportional_on_measurement = True
            
            # We call dynamic server here after the PIDs are set up
            # so the new PID values are set after the PIDs were created
            srv = Server(ThunderborgConfig, self.DynamicCallbak)
        
        self.__thunderborg = thunderborg_lib.ThunderBorg()  # create the thunderborg object
        self.__thunderborg.Init()
        if not self.__thunderborg.foundChip:
            rospy.logdebug("ThunderBorg board not found")
        else:
            # Setup board to turn off motors if we don't send a message every 1/4 second          
            self.__thunderborg.SetCommsFailsafe(True)

        # Motor velocity feedback values m/s
        self.__feedback_velocity1 = 0.0
        self.__feedback_velocity2 = 0.0
        
        # Speed request in m/s
        self.__speed_wish_right = 0.0
        self.__speed_wish_left = 0.0
        
        # Publish topics
        self.__status_pub = rospy.Publisher("main_battery_status", BatteryState, queue_size=1)
        if self.__diag_msgs == True:
            self.__diag1_pub = rospy.Publisher("motor1_diag", Vector3, queue_size=1)
            self.__diag2_pub = rospy.Publisher("motor2_diag", Vector3, queue_size=1)

        # Subscribe to topics
        self.__vel_sub = rospy.Subscriber("cmd_vel",Twist, self.VelCallback)
        self.__feedback_sub = rospy.Subscriber("tacho", tacho, self.TachoCallback)

    # Dynamic recofiguration of the PIDS
    def DynamicCallbak(self, config, level):
        self.__pid1.tunings = (config.p_param, config.i_param, config.d_param)
        self.__pid2.tunings = (config.p_param, config.i_param, config.d_param)
        return config
        
    # Function to calculate thunderborg setting from velocity
    def MotorSetting(self, vel):
        if vel == 0:
            setting = 0
        else:
            setting = (abs(vel)+self.__speed_y_intercept)/self.__speed_slope
            if vel < 0:
                setting = -(setting)
        return setting
        
    # Callback for cmd_vel message
    def VelCallback(self, msg):       
        # Calculate the requested speed of each wheel
        self.__speed_wish_right = ((msg.angular.z * self.__wheel_distance) / 2) + msg.linear.x
        self.__speed_wish_left = (msg.linear.x * 2) - self.__speed_wish_right

        # Convert speed demands to values understood by the Thunderborg. We limit to 1m/s
        motor1_speed = self.MotorSetting(self.__speed_wish_right)
        motor2_speed = self.MotorSetting(self.__speed_wish_left)
        
        if self.__use_pid == True:
            # Using the PID so update set points
            self.__pid1.setpoint = abs(motor1_speed)
            self.__pid2.setpoint = abs(motor2_speed)
        else:
            # Update the Thunderborg directly
            self.__thunderborg.SetMotor1(motor1_speed)
            self.__thunderborg.SetMotor2(motor2_speed)

            if self.__diag_msgs == True:
                motor1_state = Vector3()
                motor1_state.x = self.__speed_wish_right
                motor1_state.y = self.__feedback_velocity1
                motor1_state.z = motor1_speed
                motor2_state = Vector3()
                motor2_state.x = self.__speed_wish_left
                motor2_state.y = self.__feedback_velocity2
                motor2_state.z = motor2_speed
                self.__diag1_pub.publish(motor1_state)
                self.__diag2_pub.publish(motor2_state)

    # Callback for tacho message
    def TachoCallback(self, msg):
        # Store the feedback values as velocity m/s
        self.__feedback_velocity1 = (msg.rwheelrpm/60.0)*self.__wheel_circumfrence
        self.__feedback_velocity2 = (msg.lwheelrpm/60.0)*self.__wheel_circumfrence
        
        # TODO for ODOM use
        # self.__speed_wish_right sign to determine direction of feedback for 1
        # self.__speed_wish_left sign to determine direction of feedback for 2
       
    # Publish the battery status    
    def PublishStatus(self):
        battery_state = BatteryState()
        # Read the battery voltage
        try:
            battery_state.voltage = self.__thunderborg.GetBatteryReading()        
            self.__status_pub.publish(battery_state)
        except:
            rospy.logwarn("Thunderborg node: Failed to read battery voltage");

    # Update the PIDs and set the motor speeds
    def RunPIDs(self):
        if self.__use_pid == True:
            # Update PIDs and get next value
            pid1_output = self.__pid1(self.MotorSetting(self.__feedback_velocity1))
            pid2_output = self.__pid2(self.MotorSetting(self.__feedback_velocity2))
            
            # Check if demand is below inertia level. The pid lower level is set to this value
            # to stop the pid going below this level when responding to an overshoot and causing
            # a motor stall
            if self.__pid1.setpoint < self.__inertia:
                motor1_speed = 0    
            # else check direction required
            elif self.__speed_wish_right < 0:
                motor1_speed = -(pid1_output)
            else:
                motor1_speed = pid1_output
            
            if self.__pid2.setpoint < self.__inertia:
                motor2_speed = 0
            elif self.__speed_wish_left < 0:    
                motor2_speed = -(pid2_output)
            else: 
                motor2_speed = pid2_output
            
            # Set motor speeds
            self.__thunderborg.SetMotor1(motor1_speed)
            self.__thunderborg.SetMotor2(motor2_speed)
            
            if self.__diag_msgs == True:
                motor1_state = Vector3()
                motor1_state.x = self.__pid1.setpoint
                motor1_state.y = self.MotorSetting(self.__feedback_velocity1)
                motor1_state.z = motor1_speed
                motor2_state = Vector3()
                motor2_state.x = self.__pid2.setpoint
                motor2_state.y = self.MotorSetting(self.__feedback_velocity2)
                motor2_state.z = motor2_speed
                self.__diag1_pub.publish(motor1_state)
                self.__diag2_pub.publish(motor2_state)
    
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
            
        # Run the PIDs
        tbn.RunPIDs()
        
        rate.sleep()
               

if __name__ == '__main__':
    main(sys.argv)
