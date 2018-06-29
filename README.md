#Autonomous Tracked Robot on the RPi platform

![Current Version front](https://github.com/Kheiden/RPi-tankbot/tree/master/documentation/photos/Current_Version_front.jpg)

server/
- Code related to processing the video stream
- Executed on the server

local/
- Code related to low level functions
- Executed on the RPi


Goal:
The goal of this project is to create an autonomous tracked robot.

Success Factors:
- Build a tracked robot with stereo vision. [Complete]
- The cameras must be able to perform object recognition and log the objects with a timestamp to a database.
- The robot needs to have depth of field calculations from the stereo cameras which will be used as a sensor for object avoidance.
- The robot needs to be able to be both remotely controlled and autonomous.
- In autonomous mode, the robot will need to be able to navigate any surrounding without getting stuck
- The robot will need to be able to "return to home" within a few cm of where "home" is set.
- The robot will be able to create a 3d representation of its surroundings

Hardware Stretch Goal:
- The robot will charge wirelessly and must be able to charge itself when low on power.
