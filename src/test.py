#!/usr/bin/env python
import thunderborg_lib
import time

# Run with different values 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0
# WARNING ensure your robot as room to run for 5 seconds at the set speed or channge the sleep time!
speed = 0.5

TB = thunderborg_lib.ThunderBorg()  # create the thunderborg object
TB.Init()

if not TB.foundChip:
    print("ThunderBorg board not found")
else:
    # Set both motor speeds  
    TB.SetMotor1(speed)
    TB.SetMotor2(speed)
    
    time.sleep(5)

    TB.SetMotor1(0.0)
    TB.SetMotor2(0.0)
    
    
    
    

