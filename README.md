# 1.Overview

ROS package for Kobot -- Heteregoneous swarm robot platform.

Kobot package is actively used and tested on RPi 3B+ running Ubuntu Mate 18.04, ROS Melodic and RPi Zero W running Raspbian Buster and ROS Melodic.

Additionally, this repository contains dotfiles (.vimrc, .zshrc, .tmux.conf), shell scripts and plain hardware test scripts that are used by Kobot robots.

Currently, 3 different swarm robots are extended from the Kobot platform:
- Kobot-W (Wheeled robot, equipped with IR floor sensors, 9-Axis IMU, Xbee for local communication) [Design Link](https://a360.co/2AbiR6Z)
- Kobot-F (Flying robot using Ryze Tello as the base) [Desing Link](https://a360.co/3BwyYGh)
- Kobot-T (Tracked Robot using Zumo from Pololu as the base, equipped with a 2-D.o.F. compliant gripper) [Design Link will be available soon]
## SD Card Images
### Raspberry Pi 3B/3B+ 
[SD Card Image](https://drive.google.com/file/d/1Ou2V7OFRBmUFKtcfE01nRIsgRfbQocXO/view?usp=sharing) of the Kobot-W running Ubuntu Mate 18.04 and ROS Melodic 

### Raspberry Pi Zero W 
[SD Card Image](https://drive.google.com/file/d/1YUq0BFDZqvFaTTxi0CF9Gvh6_W8bduPv/view?usp=sharing) of the Kobot-F running Raspbian Buster and ROS Melodic
Image of the Kobot-T running Raspbian Buster and ROS Melodic (In-progress)
## Branches
Branches are used for developing extended robot specific features until they are ready to be merged with the main branch. Maintainers of the branches are as following:
- main (<cembi@metu.edu.tr>)
- kobot_w (<cembi@metu.edu.tr>)
- kobot_f (<cembi@metu.edu.tr>,<mehmet.sahin1001@gmail.com>)
- kobot_t (<egur@metu.edu.tr>)

## Hardware Architecture
![Alt text](img/kobot_hw.png?raw=true "Title")
## Software Architecture
![Alt text](img/kobot_sw.png?raw=true "Title")

## Dependencies
If you are not using the SD Card images and would like to use kobot_ros package for other purposes, there are quite a few required dependencies which are given below (these are not updated regularly):
- cv_bridge (need to build as a seperate catkin workspace)
- camera_info_manager (required by ROS nodes)
- tf (required by ROS nodes)
- Python 3.6 or later version (not the default interpreter for ROS Melodic)
- opencv-contrib-python (need to build from source for RPi Zero and 3B+)
- mpu9250_jmdev (handling i2c comm. with MPU9255 9-Axis IMU)
- adafruit_neopixel (handling comm. w/ neopixel led ring by using one of the GPIO pins)
- pyserial (handling the serial comm. w/ Xbee RF Module)
- smbus2 (handling all the i2c communication)
- numpy (handling linear algebra)
- RPi.GPIO (handling basic GPIO operations such as digital write, read)

## Installation
There are no installation instructions for the stand-alone kobot package. Recommended way is using one of the SD Card images.

# 2.Nodes
This repository contains two types of nodes:

- High-level beahvioural nodes to control the swarm behaviour of the robots 
  - flocking [Paper Link](https://link.springer.com/article/10.1007/s11721-008-0016-2) i.e. Swarm members perform alignment, avoidance and cohesion using robot, obstacle detection, range of the objects, heading with resepect to an absolute reference and non-holonomic velocity control [Video Link](https://drive.google.com/file/d/1GWaZFbCjzwua60wWCDh6ZlnNDX_1mw3i/view?usp=sharing)
  - alignment, i.e. modified flocking algorithm where swarm members align their headings w.r.t. a common orientation reference
  - wall_following i.e. modified flocking algorithm where swarm members follow walls instead of avoiding perpendicularly
  - dispersion_avoidance, i.e. modified flocking algorithm where Kobot-F robots disperse in z-axis (altitude), avoid obstacles [Video Link](https://drive.google.com/file/d/1Qoc8hAuECTHkP4aB6Oa5qzmxWwkxl2Rp/view?usp=sharing)
  - beeclust [Paper Link](https://www.tandfonline.com/doi/full/10.1080/13873954.2011.601420) i.e. Swarm members aggregate on a cue denoted by high IR reflectivity (other abstractions are possible), avoid obstacles by random walk and wait in the cue when collide with another robot inside cue. Duration is determined by the reflectivity
  - lba (landmark-based aggregation) [Paper Link](https://journals.sagepub.com/doi/full/10.1177/1059712320985543?casa_token=q_RrkzviblcAAAAA%3AwPuCq_PO9i7I9j5rSXD0A29FqiZqERGew60VEbs25JeTriCf1bpEY-u6w7FucfbQ94Md0O6SmLgk) i.e. Extended version of beeclust where swarm members learn and store the paths from landmarks to the cue (landmarks are abstracted by ArUco markers [Paper Link](https://www.researchgate.net/publication/325787310_Speeded_Up_Detection_of_Squared_Fiducial_Markers) detection by OpenCV) [Video Link](https://drive.google.com/file/d/1ONbTPkSrczcrn4SOYSlqRtR3J9pmwRIJ/view?usp=sharing)
  - rl_lba (landmark-based aggregation using reinforcement learning), i.e. Extended version of lba exploring, exploiting an action space when a landmark in the environment is detected
- Sub-system driver nodes abstracting the hardware using ROS messages and subscriber/publisher architecture. 
  - common nodes for all extended robots
    - range_n_bearing
    - landmark_pose_estimator
    - pose_controller
    - heading
    - local_communication
    - battery_monitor
  - kobot_w, kobot_t specific nodes
    - teleop_key
    - cmd_vel2motors
    - differential_driver
    - wheel_odometry
    - landmark_detector
    - mpu9255
    - floor_sensor
  - kobot_f specific nodes
    - tello_driver
## Messages
Custom ROS messages used by the Kobot sub-systems and behavioural nodes:
- range_n_robot, i.e. reading of an individual sensor of the range and bearing board. Robot field is a boolean which is true for robots and false for all other objects. Range field is the range reading corresponding to the sensor.
- range_n_bearing, i.e. array of all sensor of the range_n_bearing board.
- virtual_sensor, i.e. array of sensor readings.
- floor_sensor, i.e. array of individual IR floor sensor readings.
- landmark_pose, landmarks are abstracted by using Aruco markers, 
## Parameter Files
Also two types of parameter files in .yaml format are used to configure the nodes:
- Behavioural parameters
  - flocking
  - alignment (modified flocking beta=0, u_max=0)
  - wall_following (modified flocking avoid_angle = 90)
  - lba (modified beeclust)
  - beeclust (modified lba d_landmark = 0)
  - rl_lba (modified lba)
- Calibration files, these files are specific to robots and stored locally hence, they are ignored by git.
  - IMU (gyro bias, magnetometer hard-iron, soft-iron biases)
  - Camera (intrinsic parameters, calibrated with a Charuco Board)
  - Odometry (left, right wheel radii and axle distance)
- Controller
  - Velocity Controller (PID gains, velocity limits, control loop frequency)
  - Pose Controller (PID gains, error-window, max. translational velocity)

| Flocking Parameter        | Explanation          | Default Value  |
| ------------- |:-------------:| -----:|
| beta      | Weight of proximal control to heading alignment | 1.5 |
| u_max      | Max. translational velocity      |   0.1 m/s |
| sigma_des_robot | Equilibrium distance from robots      |    350 mm |
| sigma_des_obstacle | Equilibrium distance from obstacles      |  1000 mm |
| avoid_angle |  Angle of avoidance  |  180 deg |

| Aggregation Parameter        | Explanation          | Default Value  |
| ------------- |:-------------:| -----:|
| w_max      | Max. wait duration inside the cue| 120 s |
| u_max      | Max. translational velocity      |   0.1 m/s |
| d_landmark | Max. detection distance for landmarks |    500 mm |
| d_obs | Max. detection distance for obstacles|  100 mm |
| d_rob | Max. detection distance for robots|  250 mm |

| Calibration Parameter        | Explanation          | Default Value  |
| ------------- |:-------------:| -----:|
| w_max      | Max. wait duration inside the cue| 120 s |
| u_max      | Max. translational velocity      |   0.1 m/s |
| d_landmark | Max. detection distance for landmarks |    500 mm |
| d_obs | Max. detection distance for obstacles|  100 mm |
| d_rob | Max. detection distance for robots|  250 mm |

| Controller Parameter        | Explanation          | Default Value  |
| ------------- |:-------------:| -----:|
| w_max      | Max. wait duration inside the cue| 120 s |
| u_max      | Max. translational velocity      |   0.1 m/s |
| d_landmark | Max. detection distance for landmarks |    500 mm |
| d_obs | Max. detection distance for obstacles|  100 mm |
| d_rob | Max. detection distance for robots|  250 mm |
## Launch Files
Launch files are used for running multiple nodes, loading parameters to the ROS's parameter server and handling individual namespaces.
- Sub-system launch files
  - heading
  - move
  - imu
- Behaviour launch files
  - flocking
  - aggregation
  - dispersion_avoidance
- Calibration launch files
  - camera
  - imu
  - odometry
  - floor_sensor
# 4.Work in Progress
- Kobot-T sub-system drivers 
- flocking behaviour for Kobot-F
- Kobot-H and Kobot-L (holonomic wheeled and legged swarm robots) conceptual design
# 5.License
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
# 6.Additional Links
## Related Repositories
![Alt text](img/software_architecture.png?raw=true "Title")
- kobotArduino [Link] TODO
- BpVL53L1 [Link] TODO
- kobot_base [Link] TODO
## Related Webpages
- ROMER -- Instuition where Kobot platform is deleoped [Webpage](https://romer.metu.edu.tr/tr)
- Kovan Research Lab -- Laboratory where Kobot platform is developed [Webpage](https://kovan.ceng.metu.edu.tr/)

