Group 08: Hans Boehme, Christopher Zeiser, David Full

=======
Basic setup for the TAS stack

This repository has two branches: master and simulation.	
	1. Branch "master": for working on the real car
	2. Branch "simulation": for working on gazebo simulation

===================================================================================================

1. Branch "master":

Task 1: 
	1. To record a map (using hector mapping and starting the hardware and odom node):
		- `roslaunch tas run1.launch`
	2. Complete the lap.
	3. To save the map by using the service "map_saver" of the package "map_server":
		- `roslaunch tas run2.launch` and then
		- `bash run2.sh`
	4. Close the terminal or kill all running ros nodes by using ‘ctrl + c’.
	5. To run all necessary nodes for autnonomous driving/parking:
		- `roslaunch tas run3.launch`
	6. To choose the right car, set parameter “car”:
		- `rosparam set car 1` (Vettel)
		- `rosparam set car 2` (Futurama)
		- `rosparam set car 3` (Gerthy)
	 	- `rosparam set car 4` (Kitt)
	7. Press B-Button on wii-mote (for about half a second) to plan trajectory.
	8. Press C-Button to start autonomous driving.
	9. To change forward and backward speed and the steering-angle offset by x,y,z:
		- `rosparam set add_forward_speed x`
		- `rosparam set add_backward_speed y`
		- `rosparam set add_steering_offset z`

Task 2:
	1. Run step 5 and 6 of Task 1 if not already done.
	2. Place car as described in task description for parallel parking.
	3. Press and hold A-Button on wii-mote until the car has reached its final parking position.
	4. Release A-Button to leave parking position.
	

===================================================================================================
Contributions:
	Christopher Zeiser:
	
	Hans Boehme:
		- Map saving with shell script
		- User friendy mode control with wii-mote (A-Button for parking; B-Button for trajectory planning)
		- 
	
	David Full:
