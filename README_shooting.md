## ﻿README FILE FOR AIMING

To start, please allow me to apologise for the frankenstein of a code we created. 
Two of us (Jareth and Justin) were working independently on separate parts of this code and we didn't plan for integration well. This is why there are some redefined functions that were renamed to allow us to use both of our own versions of the function.

Also, please take note that the file that we used for the final run was **aiming_algorithm3.py**
The code has since been amended to remove most irrelevant details and comments. The cleaned up version is **aiming_algorithm4.py**.



### AIMING ALGORITHM OVERVIEW

The overall flow of our algorithm is as follows:

1. The nodes, devices and constants are first set up and the detection algorithm will run in the background

2. If the IR target is detected (ie. if the thermal camera detects a temperature above the user-specified threshold), the navigation node will be instructed to stop via a 'shoot' topic and the aiming node will take over.

3. Aiming then commences:
	a) The turtlebot checks if the target is straight ahead. If not, it will rotate 		clockwise or anti-clockwise until the target is directly in front of the turtlebot
        b) The turtlebot will then move forward until it is an appropriate distance away from 		the target
        c) A second round of rotating to center the target will take place to enhance acuracy
        
4. Shooting commences. The tilting servo will first be set to an angle of 65 degrees (which corresponds to an elevation of 25 degrees) and the pusher servo will turn on to shoot the first ball by pushing it towards the flywheel. This process is repeated at angles of 75 and 85 degrees.

5. Once the shooting finishes, the navigation node will be instructed to take over once more via the 'shoot' topic.



### ADDITIONAL NOTES ON HARDWARE

* This code was written with the expectation that the flywheel would be on throughout the entire maze mapping and shooting exercise. Hence, there is no GPIO control for the DC motor

* The thermal camera was mounted on the front of the turtlebot, between layers 2 and 3. Since the thermal camera cannot be angled, it cannot be used to approximate the distance from a target that is elevated too high. Hence, two conditions were used to approximate a reasonable distance from the target:

1) A space\_temp was set. If the maximum temperature detected by the camera is higher than this space\_temp, the bot is likely close enough for the balls shot to hit the target

2) A scan\_temp was set. If the maximum temperature detected by the the camera is lower than this scan\_temp, it is likely that the target is elevated and the bot cannot detect the target at all. In this case, the bot is likely close enough that the angle between the target and the camera is too great for the camera to detect. 



### ADDITIONAL NOTES ON SOFTWARE

* A brief explanation of the use of each function, along with an explanation of the parameters and return values are included in a docstring at the beginning of each function.



### ADDITIONAL TEST FILES

* The test files in this repository are simple tests from when I was testing each of the devices independently. Not all of them can be executed without errors because some of them were used to determine if something was feasible or not. The files and their descriptions are outlined in the table below

|File                |Description                                                                         |
|--------------------|------------------------------------------------------------------------------------|
|continuous_test.py  |Test the functionality of the continuous servo                                      |
|flywheel_test.py    |Test the DC motor with the mosfet and GPIO pins (failed because mosfet burnt)       |
|fun\_with\_servos.py|Test controlling the angle of servos                                                |
|pitch_test.py       |Test aiming with tilting servo                                                      |
|pixels_test.py      |Test the AMG8833 thermal camera functionality using AMG's old library               |
|pixels_test2.py     |Test the AMG8833 thermal camera functionality using AMG's newer library             |
|rotation_testing.py |Test controlling the clockwise and anti-clockwise rotation of the turtlebot         |
|shooter_test.py     |Test the shooting portion without the aiming portion                                |








