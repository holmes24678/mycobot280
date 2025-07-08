
# mycobot280 + ROS + Moveit2Stack + YOLO

This Projects Demonstrates the configuration of mycobot288, a Six- Degrees of Freedom Robot with ROS and Moveit2Stack. Attaching Camera to the Robot Arm to perform Object Detection with YOLO by Simulating the Whole thing.

### Setup Environment
#### 1. Create ROS Workspace on your Linux Machine
```
$ mkdir -p mycobot_ws/src
```
#### 2. Change Directory to source
```
$ cd mycobot_ws/src
```
#### 3. Clone the the github Repository
```
$ git clone https://github.com/holmes24678/mycobot280
```
#### 4. Build the Workspace and source it(add to .bashrc)
```
$ cd ../

$ colcon build

$ source install/setup.bash
```
### 1. Simulating in Gazebo, RVIZ and configuration with Moveit2Stack
In this Section, we will see how to Simulate the mycobot280 in RVIZ, gazebo and later configuration with Moveit2Stack

#### 1. Launching the Robot in RVIZ
```
$ ros2 launch mycobot280 display.launch.py
```
###### The command launches the robot in RViZ. You will see the output as follows

![Screenshot from 2025-07-08 13-12-10](https://github.com/user-attachments/assets/60f3c26e-b1aa-4f38-8823-8dcda7d05675)

#### 2. Launching the Robot in Gazebo
```
$ ros2 launch mycobot280 gazebo.launch.py
```
###### The command launches the robot in Gazebo. You will see the output as follows
![Screenshot from 2025-07-08 13-13-07](https://github.com/user-attachments/assets/5e6fed5b-5377-4eaa-a791-642ffb4df4ea)

#### 3. configuration of Robot in Moveit2Stack
```
$ ros2 launch mycobot280 moveit.launch.py
```
###### The command launches the robot in RViZ. You will see the output as follows

![Screenshot from 2025-07-08 13-14-00](https://github.com/user-attachments/assets/dfe22343-89ea-4d8d-a8d7-de482283ff2c)
![Screencastfrom2025-07-0813-15-07-ezgif com-speed](https://github.com/user-attachments/assets/2599a9d1-e8fd-4dae-b573-94740eedc170)


### 2. Implementing Forward and Inverse Kinematics
In this section we will implement Forward Kinematics and Inversekinematics using ROS2 Nodes. Created two different nodes one will send joint_angles to perform Manipulation and another will send end_effector pose to perform Manipulation

#### 1. Performing Forward Kinematics. Launch the robot in gazebo and moveit Respectively by using the commands(Aleready Given)

```
$ ros2 run mycobot280 send_joint_angles_node
```
This will perform the forward Kinematics by sending the joint_angles to moveit for Manipulation. you will get the following output

![FK-ezgif com-speed](https://github.com/user-attachments/assets/3fa818bb-0708-45d0-bf97-6b84a4931dd9)

#### 2. Performing Inverse Kinematics. Launch the robot in gazebo and moveit Respectively by using the commands(Aleready Given)

```
$ ros2 run mycobot280 send_pose_node
```
This will perform the Inverse Kinematics by sending end_effector pose to manipulate the Robotic arm. you will get the following output.
![IK-ezgif com-speed](https://github.com/user-attachments/assets/62acf3cd-8aa5-4fc9-b74b-5d4988329f1e)


### 3. Performing Object Detection using YOLO
In this section, we will implement object Detection using YOLO Library. A node was created that subscribes to /image_raw topic to get the image data and peforms the object Detection using YOLO Later the it will published to ros2 topic

#### 1. To perform Object Detection. Launch the Robot in Gazebo using commands (Aleready Givem). after running using Resource Spawner in gazebo to place object infront of robot for object Detection. in this a tortoise was placed but its was unable to classify the object. (It will work better in real world Simulation). Run the following command. Make sure to Make the detect.py executable
```
$ ros2 run mycobot280 detect.py 
```
#### 2. To see detected and undetected image use rqt_image_view by using the following command
```
$ ros2 run rqt_image_view rqt_image_view
```
you will get the following output

![Screencastfrom2025-07-0814-35-32-ezgif com-speed](https://github.com/user-attachments/assets/db13b80d-23a4-4ce7-8ff2-bbac7f544cc9)

