#!/usr/bin/env python
import sys
import rospy
import thunderborg_lib
import tf
from simple_pid import PID  # See https://pypi.org/project/simple-pid
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Vector3, Twist, Point, Pose, Quaternion
from nav_msgs.msg import Odometry
from dynamic_reconfigure.server import Server
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Empty
from tacho_msgs.msg import tacho
from thunderborg.cfg import ThunderborgConfig
from math import cos, sin

RATE = 20

class ThunderBorgNode:
    def __init__(self):
        # Read values from parameter server
        self.__use_pid = rospy.get_param('/pid/use_pid', False)
        self.__wheel_distance = rospy.get_param('/wheels/distance', 0.23)
        self.__wheel_circumfrence = rospy.get_param('/wheels/circumfrence', 0.34)
        self.__speed_slope = rospy.get_param('/speed/slope', 1.5)
        self.__speed_y_intercept = rospy.get_param('/speed/y_intercept', 0.4)
        self.__inertia = rospy.get_param('/pid/inertia_level', 0.0)
        self.__diag_msgs = rospy.get_param('/speed/motor_diag_msg', False)
        
        if self.__use_pid == True:
            # Configure the PIDs. Dummy values of zero, the actual values we be set when
            # we start the dynamic reconfiguration server below.
            self.__pid1 = PID(0.0, 0.0, 0.0, setpoint=0)
            self.__pid1.sample_time = 0.05
            # Limit the pid range. The PID will only work in positive values
            self.__pid1.output_limits = (self.__inertia, 1.0)
            
            self.__pid2 = PID(0.0, 0.0, 0.0, setpoint=0)
            self.__pid2.sample_time = 0.05
            self.__pid2.output_limits = (self.__inertia, 1.0)
            
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
        self.__feedback_velocity_right = 0.0
        self.__feedback_velocity_left = 0.0
        
        # Last motor direction
        self.__fwd_right = True
        self.__fwd_left = True
        
        # Speed request in m/s
        self.__speed_wish_right = 0.0
        self.__speed_wish_left = 0.0
        
        # Publish topics
        self.__status_pub = rospy.Publisher("main_battery_status", BatteryState, queue_size=1)
        self.__odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        if self.__diag_msgs == True:
            self.__diag1_pub = rospy.Publisher("motor1_diag", Vector3, queue_size=1)
            self.__diag2_pub = rospy.Publisher("motor2_diag", Vector3, queue_size=1)

        # Setup tf broadcaster
        self.__odom_broadcaster = tf.TransformBroadcaster()

        # ODOM values
        self.__odom_x = 0.0
        self.__odom_y = 0.0
        self.__odom_th = 0.0
        self.__last_odom_time = rospy.Time.now()

        # Subscribe to topics
        self.__vel_sub = rospy.Subscriber("cmd_vel",Twist, self.VelCallback)
        self.__feedback_sub = rospy.Subscriber("tacho", tacho, self.TachoCallback)
        self.__odom_reset_sub = rospy.Subscriber("/commands/reset_odometry", Empty, self.ResetCallback)

    # Dynamic recofiguration of the PIDS
    def DynamicCallbak(self, config, level):
        self.__pid1.tunings = (config.p_param, config.i_param, config.d_param)
        self.__pid2.tunings = (config.p_param, config.i_param, config.d_param)
        return config
        
    # Function to calculate thunderborg setting from velocity
    def MotorSetting(self, vel):
        if vel == 0.0:
            setting = 0.0
        else:
            setting = (abs(vel)-self.__speed_y_intercept)/self.__speed_slope
            if vel < 0.0:
                setting = -(setting)
        return setting
        
    # Callback for cmd_vel message
    def VelCallback(self, msg):       
        # Calculate the requested speed of each wheel
        self.__speed_wish_right = ((msg.angular.z * self.__wheel_distance) / 2) + msg.linear.x
        self.__speed_wish_left = (msg.linear.x * 2) - self.__speed_wish_right

        # Convert speed demands to values understood by the Thunderborg.
        motor1_value = self.MotorSetting(self.__speed_wish_right)
        motor2_value = self.MotorSetting(self.__speed_wish_left)
        
        if self.__use_pid == True:
            # Using the PID so update set points
            self.__pid1.setpoint = abs(motor1_value)
            self.__pid2.setpoint = abs(motor2_value)
            
            if motor1_value == 0.0:
                # Leave flag as is
                pass                            
            elif motor1_value < 0.0:
                self.__fwd_right = False
            else:
                self.__fwd_right = True

            if motor2_value == 0.0:
                # Leave flag as is
                pass                            
            elif motor2_value < 0.0:
                self.__fwd_left = False
            else:
                self.__fwd_left = True
            
        else:
            # Update the Thunderborg directly   
            self.__thunderborg.SetMotor1(motor1_value)
            self.__thunderborg.SetMotor2(motor2_value)

            if self.__diag_msgs == True:
                motor1_state = Vector3()
                motor1_state.x = self.__speed_wish_right
                motor1_state.y = self.__feedback_velocity_right
                motor1_state.z = motor1_value
                motor2_state = Vector3()
                motor2_state.x = self.__speed_wish_left
                motor2_state.y = self.__feedback_velocity_left
                motor2_state.z = motor2_value
                self.__diag1_pub.publish(motor1_state)
                self.__diag2_pub.publish(motor2_state)

    # Callback for tacho message
    def TachoCallback(self, msg):
        # Store the feedback values as velocity m/s
        self.__feedback_velocity_right = (msg.rwheelrpm/60.0)*self.__wheel_circumfrence
        self.__feedback_velocity_left = (msg.lwheelrpm/60.0)*self.__wheel_circumfrence

    # Callback for odometry reset command
    def ResetCallback(self, msg):
        self.__odom_x = 0.0
        self.__odom_y = 0.0
        self.__odom_th = 0.0
       
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
            # Update PIDs and get next value.
            if abs(self.__feedback_velocity_right) <= self.__inertia:
                pid1_output = self.__pid1(self.__inertia)
            else:
                pid1_output = self.__pid1(self.MotorSetting(abs(self.__feedback_velocity_right)))

            if abs(self.__feedback_velocity_left) <= self.__inertia:
                pid2_output = self.__pid2(self.__inertia)
            else:                
                pid2_output = self.__pid2(self.MotorSetting(abs(self.__feedback_velocity_left)))            
            
            if pid1_output <= self.__inertia:
                motor1_speed = 0.0
            elif self.__fwd_right == False:
                motor1_speed = -(pid1_output)
            else:
                motor1_speed = pid1_output

            if pid2_output <= self.__inertia:
                motor2_speed = 0.0                
            elif self.__fwd_left == False:
                motor2_speed = -(pid2_output)
            else:
                motor2_speed = pid2_output                
            
            # Set motor value
            self.__thunderborg.SetMotor1(motor1_speed)
            self.__thunderborg.SetMotor2(motor2_speed)
            
            if self.__diag_msgs == True:
                motor1_state = Vector3()
                motor1_state.x = self.__pid1.setpoint
                motor1_state.y = pid1_output
                motor1_state.z = self.__feedback_velocity_right
                motor2_state = Vector3()
                motor2_state.x = self.__pid2.setpoint
                motor2_state.y = pid2_output
                motor2_state.z = self.__feedback_velocity_left
                self.__diag1_pub.publish(motor1_state)
                self.__diag2_pub.publish(motor2_state)

    # the navigation stack requires that the odometry source publishes both a 
    # transform and a nav/msgs/Odometry message.
    def PublishOdom(self):
        forward_velocity = (self.__feedback_velocity_left + self.__feedback_velocity_right)/2
        angular_velocity = 2*(self.__feedback_velocity_right - forward_velocity)/self.__wheel_distance

        # Now the x and y velocities
        velocity_x = forward_velocity * cos(angular_velocity)
        velocity_y = forward_velocity * sin(angular_velocity)

        # Compute odometry from the calculate velocities
        current_time = rospy.Time.now()
        delta_time = (current_time - self.__last_odom_time).to_sec()    # Floating point seconds
        delta_x = (velocity_x * cos(self.__odom_th) - velocity_y * sin(self.__odom_th)) * delta_time
        delta_y = (velocity_x * sin(self.__odom_th) + velocity_y * cos(self.__odom_th)) * delta_time
        delta_th = angular_velocity * delta_time

        # Add the latest calculated movement
        self.__odom_x += delta_x 
        self.__odom_y += delta_y
        self.__odom_th += delta_th

        # we need Yaw in a Quaternion
        odom_quat = quaternion_from_euler(0, 0, self.__odom_th)

        # Send the transform
        self.__odom_broadcaster.sendTransform((self.__odom_x, self.__odom_y, 0.0),
                                              odom_quat,
                                              current_time,
                                              'base_link',
                                              'odom')

        # Next publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        # The pose is specified to the odom co-ordinate frame
        odom.pose.pose = Pose(Point(self.__odom_x, self.__odom_y, 0.), Quaternion(*odom_quat)) 
        # The Twist velocities is specified to the base_link co-ordinate frame
        odom.child_frame_id = 'base_link'
        odom.twist.twist = Twist(Vector3(velocity_x, velocity_y, 0), Vector3(0, 0, angular_velocity))
        
        # Publish the message
        self.__odom_pub.publish(odom)

        self.__last_odom_time = current_time


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

        # Publish ODOM data
        tbn.PublishOdom()
          
        # Run the PIDs
        tbn.RunPIDs()
        
        rate.sleep()
               

if __name__ == '__main__':
    main(sys.argv)
