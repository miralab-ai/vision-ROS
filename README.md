# Integrated vision-ROS system utilizing Yolov7-Tiny accelerated by TensorRT engine and MAVROS 

This package contains the **trt_yolo_v7.py** node that performs the inference using NVIDIA's TensorRT engine.
After **trt_yolo_v7.py** node publishes the necessary data to **tracker_offboard.cpp**, this node makes the calculations based on its inner PID controller and generates the vehicle parameters (linear velocity and angular velocity).
This parameters are shared with MAVROS package to be converted into MAVLink message format.


https://github.com/zcelil/vision-ROS/assets/57402408/85088cf6-e0b8-4b47-83e7-aee894564832




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
