#!/usr/bin/env python
import thunderborg_lib
import time

global TB
TB = thunderborg_lib.ThunderBorg()
TB.Init()

TB.SetMotor1(-0.75)
TB.SetMotor2(0.75)
time.sleep(4.0)
TB.MotorsOff()

