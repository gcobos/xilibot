# XiliBot

A self-balancing robot with a LCD screen to show faces, and a color sensor at its bottom.

In the hardware side, this repository contains the FreeCAD model, STL pieces to print, and PCB circuit desing in KiCAD.

In the software area, it includes the robot source code, which is a modified version of the [B-Robot EVO2](https://github.com/jjrobots/B-ROBOT_EVO2),
a remote controller made in Unity3D that uses [UnityOSC](https://thomasfredericks.github.io/UnityOSC) library to send commands to the robot,
and a Web controller that uses [Blockly](https://developers.google.com/blockly/) to control the robot using visual programming.
Blockly requires to run first a web server located under [python/webserver.py](python/webserver.py).

This robot uses a Wemos D1 mini as the "brains", and it is configured to create its own access point, so that the controller connects to the robot wifi.

## Xilibot standing

![alt tag](screenshots/xilibot-standing.png)
