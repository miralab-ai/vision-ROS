## Integrated vision-ROS system utilizing Yolov7-Tiny accelerated by TensorRT engine and MAVROS 

Github repository for the paper titled as "**Open-Source Visual Target Tracking System Both on Simulation Environment and Real Drone**".

This package contains the **trt_yolo_v7.py** node that performs the inference using NVIDIA's TensorRT engine.
After **trt_yolo_v7.py** node publishes the necessary data to **tracker_offboard.cpp**, this node makes the calculations based on its inner PID controller and generates the vehicle parameters (linear velocity and angular velocity).
This parameters are shared with MAVROS package to be converted into MAVLink message format.

The official paper can be found at the following link:

https://link.springer.com/chapter/10.1007/978-3-031-52760-9_11

## Abstract
This work presents an investigation into the domain of dynamic target tracking through object detection, particularly emphasizing the context of open-source applications like PX4, ROS, and YOLO. Over the years, achieving real-time object tracking on UAVs in dynamic environments has been a formidable challenge, necessitating offline computations or substantial onboard processing resources. However, contemporary UAVs are now equipped with advanced edge embedded devices, sensors, and cameras, enabling the integration of deep learning-based vision applications. This advancement offers the prospect of directly deploying cutting-edge applications onto UAVs, thereby expanding their utility in areas such as surveillance, search and rescue, and videography. To fully harness the potential of these vision applications, a communication infrastructure interfacing with the UAV’s underneath closed controllers becomes imperative. We’ve developed an integrated visual target-tracking system that connects a flight controller unit with a graphical unit by leveraging ROS tools and open-source deep learning packages. The overall integrated system based on ROS, deep learning applications, and custom PID controllers is shared on GitHub as open-source software package in a way that benefits everyone interested: https://github.com/miralab-ai/vision-ROS.

## Example 1 - Person Tracking (Fine-tuning both lateral and longitudinal controllers)
![person_track](https://github.com/zcelil/vision-ROS/assets/57402408/bb5893c1-d558-487e-84aa-6475d7ab0a44)


## Setting up the environment

### Install dependencies

### Current Environment:

- Jetson Nano
- ROS Melodic
- Ubuntu 18.04
- Jetpack 4.5.1
- TensorRT 7+

#### Dependencies:

- OpenCV 3.x
- numpy 1.15.1
- Protobuf 3.8.0
- Pycuda 2019.1.2
- onnx 1.4.1 (depends on Protobuf)

### Install all dependencies with below commands

```
Install pycuda (takes awhile)
$ cd ${HOME}/catkin_ws/src/vision-ROS/dependencies
$ ./install_pycuda.sh

Install Protobuf (takes awhile)
$ cd ${HOME}/catkin_ws/src/vision-ROS/dependencies
$ ./install_protobuf-3.8.0.sh

Install onnx (depends on Protobuf above)
$ sudo pip3 install onnx==1.4.1
```
## Example 2 - Car Tracking (Fine-tuning longitudinal controller)
![ezgif com-gif-maker](https://github.com/zcelil/vision-ROS/assets/57402408/143d1539-0792-45bf-b463-74e322e2caff)

---

* Please also install [jetson-inference](https://github.com/dusty-nv/ros_deep_learning#jetson-inference)
* Note: This package uses similar nodes to ros_deep_learning package. Please place a CATKIN_IGNORE in that package to avoid similar node name catkin_make error
* If these scripts do not work for you, do refer to this amazing repository by [jefflgaol](https://github.com/jefflgaol/Install-Packages-Jetson-ARM-Family) on installing the above packages and more on Jetson ARM devices.
---
## Setting up the package

### 1. Clone project into catkin_ws and build it

``` 
$ cd ~/catkin_ws && catkin_make
$ source devel/setup.bash
```

### 2. Make libyolo_layer.so

```
$ cd ${HOME}/catkin_ws/src/trt_yolo_v7/plugins
$ make
```

This will generate a libyolo_layer.so file

### 3. Place your yolo.weights and yolo.cfg file in the yolo folder

```
$ cd ${HOME}/catkin_ws/src/trt_yolo_v7/yolo
```
** Please name the yolov7.weights and yolov7.cfg file as follows:
- yolov7.weights
- yolov7.cfg

Run the conversion script to convert to TensorRT engine file

```
$ ./convert_yolo_trt
```

- Input the appropriate arguments
- This conversion might take awhile
- The optimised TensorRT engine would now be saved as yolov7-416.trt

### 4. Change the class labels

```
$ cd ${HOME}/catkin_ws/src/trt_yolo_v7/utils
$ vim yolo_classes.py
```

- Change the class labels to suit your model

### 5. Change the video_input and topic_name

```
$ cd ${HOME}/catkin_ws/src/trt_yolo_v7/launch
```
- `trt_yolo_v7.launch` : change the topic_name

- `video_source.launch` : change the input format (refer to this [Link](https://github.com/dusty-nv/jetson-inference/blob/master/docs/aux-streaming.md)

   * video_source.launch requires jetson-inference to be installed
   * Default input is CSI camera

---
## Using the package

### Running the package

Note: Run the launch files separately in different terminals

### 1. Run the video_source 

```
# For csi input
$ roslaunch trt_yolo_v7 video_source.launch input:=csi://0

# For video input
$ roslaunch trt_yolo_v7 video_source.launch input:=/path_to_video/video.mp4

# For USB camera
$ roslaunch trt_yolo_v7 video_source.launch input:=v4l2://0
```

### 2. Run the yolo detector

```
# For YOLOv7 (single input)
$ roslaunch trt_yolo_v7 trt_yolo_v7.launch

```

### 3. For maximum performance

```
$ cd /usr/bin/
$ sudo ./nvpmodel -m 0	# Enable 2 Denver CPU
$ sudo ./jetson_clock	# Maximise CPU/GPU performance
```

* These commands are found/referred in this [forum post](https://forums.developer.nvidia.com/t/nvpmodel-and-jetson-clocks/58659/2)
* Please ensure the jetson device is cooled appropriately to prevent overheating

- Default Input FPS from CSI camera = 30.0
* To change this, go to jetson-inference/utils/camera/gstCamera.cpp 

``` 
# In line 359, change this line
mOptions.frameRate = 15

# To desired frame_rate
mOptions.frameRate = desired_frame_rate
``` 
---

## src/tracker_node.cpp node

This node enables users to establish a connection between the companion computer and the main flight controller (such as PX4 or ArduPilot) using the MAVROS package. The next two lines denote the subscriber and publisher functions. Should one wish to transmit processed vision data to a different computer, this node can be adapted to one's specific needs. In this scenario, the MAVROS subscriber acquires data from MAVLink to gain awareness of the vehicle's position. Subsequently, the MAVROS publisher transmits computed vehicle commands back to MAVLink, enabling the vehicle to execute corresponding actions.

```
ros::Subscriber pose_stamped = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, pose_stamped_cb);

ros::Publisher body_vel_pub  = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 1);

```

For the installation of MAVROS and MAVLink, you can found the details on [User Guide](https://docs.px4.io/main/en/)

---
## Licenses and References

### 1. TensorRT samples from [jkjung-avt](https://github.com/jkjung-avt/) 

### 2. SORT from [Hyun-je](https://github.com/Hyun-je/SORT-ros) 
