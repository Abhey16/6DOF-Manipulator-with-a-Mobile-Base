# Overview
This repository contains the source code and CAD files for a **6 Degree of Freedom Manipulator with a Mobile Base**. The manipulator is equipped with a vacuum gripper for pick and place operations and object manipulation. The project involved utilizing SolidWorks for creating a 3D model, implementing an inverse kinematics script in Python within the ROS2 framework, and validating the manipulator's performance through simulations in the Gazebo environment.
<br>
![manipulator](https://drive.google.com/file/d/1JZFe_RIiBJRFTksGy6gD7WWX8M3ZJapW/view)
<br>
## Installation
Follow these instructions to get a copy of the project up and running on your local machine for development and testing purposes.
1. Create a directory with a subfolder named **src** on your local machine.
2. Navigate into the directory using Terminal and run:
```
colcon build
```
This will convert your directory into a ROS2 Workspace

3. Navigate to the src folder and clone the repository to your local machine:
```
git clone https://github.com/Abhey16/6DOF-Manipulator-with-a-Mobile-Base.git
```
This will add ROS package in the src folder. Remember to add only the package not the CAD files(which are provided only for reference)

4. Navigate to the root of ROS2 directory and build the project using colcon:
```
colcon build
```
5. Source the environment to set up the necessary ROS2 variables:
```
source install/setup.bash
```
6. Now launch launch the Gazebo environment
```
ros2 launch manipulator gazebo.launch.py
```
7. Now run the controller script
8. For teleop script run:
```
ros2 run manipulator teleop.py
```


