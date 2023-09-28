# XILIBOT PC CONTROL python script
# Exampe 4: Controlling 2 XILIBOTS in a coreography...
# You should modify your XILIBOT arduino code to connecto to your wifi network
# XILIBOT1 is on IP 192.168.1.101 and XILIBOT 2 IP is 192.168.1.102

# author: JJROBOTS 2016
# version: 1.01 (28/10/2016)
# Licence: Open Source (GNU LGPLv3)

import time
from XILIBOT_Class import XILIBOT # Import CLASS to control XILIBOT

# XILIBOT1 initialization
myRobot1 = XILIBOT()
myRobot1.UDP_IP = "192.168.4.1"
myRobot1.mode(0)  # Normal mode. optional: PRO MODE=1

# XILIBOT1 initialization
myRobot2 = XILIBOT()
myRobot2.UDP_IP = "192.168.4.2"
myRobot2.mode(0)  # Normal mode. optional: PRO MODE=1


# Example of sequence of commands to XILIBOT:
myRobot1.servo(1)       #Move servo
myRobot2.servo(1)
time.sleep(0.25)
myRobot1.servo(0)
myRobot2.servo(0)
time.sleep(0.25)
myRobot1.servo(1)       #Move servo
myRobot2.servo(1)
time.sleep(0.25)
myRobot1.servo(0)
myRobot2.servo(0)

myRobot1.throttle(0.4)
myRobot2.throttle(0.4)
time.sleep(0.75)
myRobot1.throttle(0)
myRobot2.throttle(0)
time.sleep(1)
myRobot1.steering(0.8)   # Robot1 Turn right
myRobot2.steering(-0.8)  # Robot2 Turn left
time.sleep(2)
myRobot1.steering(0)
myRobot2.steering(0)
time.sleep(0.25)
myRobot1.steering(-0.8)  # Robot1 Turn left
myRobot2.steering(0.8)   # Robot2 Turn right
time.sleep(2)
myRobot1.steering(0)     #Stop
myRobot2.steering(0)
myRobot1.throttle(0)
myRobot2.throttle(0)



