# SDP-Robot-Control
Control for motors and arm of  EDVarka 

# Key functions useful for navigation in robot_movement.py

In robot_movement.py:
- get_own_position(): returns a utils.xy_position object with the robot's current position estimate. Origin is at its initial position and y axis is parallel to the north axis.
- get_own_yaw(): returns the robot's current estimate of tis yaw value which is its counter-clockwise rotation from north axis. Hence, this value should be zero when the robot is facing north.
- is_facing_target(target_position): Given a utils.xy_position, returns true if the robot is facing the target within an acceptable error.
- is_facing_yaw(target_yaw): Returns true if the robot is facing in the specified yaw direction, within an acceptable error.
- face_target(target_position): Repeatedly calling this function with a utils.xy_position will utilize the robot's propellers in order to turn to face the target.
- face_yaw(target_yaw): Similar to face_target, only that the PID controller aims to face in the specified direction instead of at the specified target.
- get_distance_to_target(target_position): Returns the robot's distance, in meters, from the target position.
- is_in_arms_open_distance(target_position): Returns true when the robot is close enough to the target so that it should open the arms.
- is_at(target_position): Returns true when the robot is negligibly close to the target.
- goto_xy(target_position): Probably the most useful function for navigation. Repeatedly calling this will see to it that the robot reaches the specified target position (first by turning the robot to face the target, then by moving forward the right amount).
- open_arms/close_arms: Self explanatory.


# Example usage

You can run the simulation from catkin_ws with the command `roslaunch edvarka test.launch`.
This will open webots and execute the main_controller.py script. 

You can modify the code in the main_controller, so that the robot does what you want it to for your testing purposes. For example, you can put the repeated logic it should follow into the function main_loop() and repeatedly call it from the main body of the script. 

IMPORTANT: don't forget to call robot.step(timestep) on each iteration, otherwise control is never passed to the simulation and it will appear frozen. Also, don't forget to call send_sensor_readings_to_localization(), otherwise the robot's localization will not work since there will be no data fed to the EKF.

An example of what could go into main_loop() is:
```python
    def main_loop(self):
    	targ = utils.xy_position(3,2) # target position is at (3,2) meters relative to the starting position of the robot
    	self.rm.goto_xy(targ)
    	self.robot.step(self.timestep)
    	self.send_sensor_readings_to_localization()
```
The above code should move the robot to the specified position.

If you would like to move the robot relative to its current position (e.g. 1 meter forward)  you can use the robot's current position and rotation estimate to calculate a target position, and then feed that into mc.rm.goto_xy(...) which will do the rest. (TODO: maybe add a function that generates coordinates relative to the robot's current position?)

If you need to launch webots without running the simulation, for example to modify the scene, you can run `roslaunch edvarka webots.launch`
