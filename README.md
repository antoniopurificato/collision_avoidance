# ROBOT PROGRAMMING
Repository of the code for the project of Robot Programming 2021/22,"La Sapienza" University of Rome.<br/>
Collision avoidance project.

### Setup
0. Go in the ```src``` folder of your catkin workspace.

1. Download the C++ teleop_twist_keyboard package to move the robot in the environment using the keyboard.
  ```
  git clone https://github.com/methylDragon/teleop_twist_keyboard_cpp.git
  ```
2. Download the avoidance package.
 ```
git clone https://github.com/antoniopurificato/collision_avoidance.git
 ```
3. Build the project.
 ```
cd .. && catkin build && source devel/setup.bash
 ```

### Execution
0. Go in the catkin workspace.

1. Start the roscore.
  ```
  roscore
  ```
2. Open a different window and run the teleop_twist_keyboard.
 ```
  source devel/setup.bash && rosrun teleop_twist_keyboard_cpp teleop_twist_keyboard cmd_vel:=cmd_vel_call
  ```
3. Open a different window and run the avoider.
```
source devel/setup.bash && rosrun collision_avoidance collision_avoider
```
4. Open a different window and run the stage.
```
source devel/setup.bash && cd src/srrg2_configs/navigation_2d/ &&rosrun stage_ros stageros cappero_laser_odom_diag_obstacle_2020-05-06-16-26-03.world
```
Come back to the terminal window with the teleop_twist_keyboard active and use the keyboard to move the robot.

### TODO:
- [x] Create a ROS node;
- [x] Complete the subscriber for the velocity;
- [x] Implement a method to switch from DH parameters to a rotation matrix related to the robot reference frame;
- [x] Complete the subscriber for the laser scan;
- [x] Write the report;
- [x] Write the setup;
- [ ] Test and bug fixing;
