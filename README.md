# thunderborg
ROS package to control the ThunderBorg motor controller board and to generate the ODOM.

## Running the Node
Once you have the node built you can run the rodney robot by launching the rodney.launch file.

## Node Information
Topics:
* `cmd_vel`:  
  Subscribes using `geometry_msgs/Twist` A request containing the current linear and angular velocity command
  
* `tacho`:  
  Subscribes using `tacho_msgs/tacho` A message with the current left and right motor rpm
  
* `main_battery_status`:  
  Publishes using `sensor_msgs/BatteryState` Message containing the state of the main battery
  
* `raw_odom`:  
  Publishes using `nav_msgs/Odometry` Message containing the raw odometry data calculated from motor encoders
  
* `motor1_diag`:  
  Publishes using `geometry_msgs/Vector3` Message contaning diagnostic information for the right hand motor. This message is only published if the parameter server paramter `~speed/motor_diag_msg` is true
  
* `motor2_diag`:  
  Publishes using `geometry_msgs/Vector3` Message contaning diagnostic information for the left hand motor. This message is only published if the parameter server paramter `~speed/motor_diag_msg` is true

Parameters:
* `~pid/use_pid`: A flag indicating if the PID should be used to contorl the motors. If True the PID is used. Default value = False
* `~wheels/distance`: A value giving the distance in metres between the wheels. Default = 0.23
* `~wheels/circumfrence`: A value giving the circumference of the wheels in metres. Default = 0.34
* `~speed/slope`: A value giving the slope of the graph used to convert motor velocity to thunderborg motor value. Default = 1.5
* `~speed/y_intercept`: A value giving the Y intercept of the graph used to convert motor velocity to thunderborg motor value. Default = 0.4
* `~pid/inertia_level`: A thunderborg motor value at which below the value the motors can't move the robot. Default = 0.0
* `~speed/motor_diag_msg`: A flag indicating if the motor diagnostic messages should be published. Default = False
* `~p_param`: The PID P parameter. can be configured dynamically. Default = 0.5
* `~i_param`: The PID I parameter. can be configured dynamically. Default = 0.9
* `~d_param`: The PID D parameter. can be configured dynamically. Default = 0.006

