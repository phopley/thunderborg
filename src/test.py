#!/usr/bin/env python
# Copyright 2019 Philip Hopley
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not 
# use this file except in compliance with the License. You may obtain a  copy
# of the License at
#
#  http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software 
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.
#
# Run with different values 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0
# WARNING ensure your robot as room to run for 5 seconds at the set speed or channge the sleep time!
import thunderborg_lib
import time

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
    
    
    
    

