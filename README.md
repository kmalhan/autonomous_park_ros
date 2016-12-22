## Autonomous Parking Spot Locator and Park System for Empty Parking Lot

This repository contains source files for ECE495 Final Project (Winter 2016)

#### Contributors
- Kazumi Malhan

### Steps to Setup Environment
1. Clone ece495_final_project from Bitbucket repository  

2. Place following lines in .bachrc  
	```bash
	  export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:"$(rospack find ugv_course_gazebo)/models":"$(rospack find project_gazebo)/models"  
	```
3. Install Teb Local Planner package  
	```bash
	sudo apt-get update
        ```   
        ```bash
	sudo apt-get install ros-indigo-teb-local-planner  
	```   
	This installed all dependent packages. One critical package is SuiteSparse package located in /usr/include/suitesparse  

4. Feature2d needs additional package  
	```bash
	sudo add-apt-repository --yes ppa:xqms/opencv-nonfree
        ```   
        ```bash  
	sudo apt-get update  
        ```   
        ```bash
	sudo apt-get install libopencv-nonfree-dev  
	```   
5. catkin_make

>Note
>
>- When catkin_make, it may complains that lane_extractConfig.h is not found. In this case, please comment out feature2d and lane_extract node of lane_extract package on CMakeLists.txt, and catkin_make. After header file is generated, un-comment sections and catkin_make again.  

>- Make sure ros-indigo-gazebo-plugins are installed.  

### Launch the Demo
```bash
roslaunch master_control final_project.launch
```

All dependent ugv_course_code is added to the repository. Sensors on audibot is modified from original code.

### Nodes on Production
>- odom_node
>- car_nav_node
>- master_node
>- set_nav_point
>- find_parking
>- lane_extract

### Nodes not used
>- feature2d
>- vehicle_control
>- bicycle_state_stapce

### Nodes under development
>- img_transform_node
>- img_transform
