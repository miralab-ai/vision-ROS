# Integrated vision-ROS system utilizing Yolov7-Tiny accelerated by TensorRT engine and MAVROS 

This package contains the **trt_yolo_v7.py** node that performs the inference using NVIDIA's TensorRT engine.
After **trt_yolo_v7.py** node publishes the necessary data to **tracker_offboard.cpp**, this node makes the calculations based on its inner PID controller and generates the vehicle parameters (linear velocity and angular velocity).
This parameters are shared with MAVROS package to be converted into MAVLink message format.

## Person Tracking (Fine-tuning both lateral and longitudinal controllers)
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
## Car Tracking (Fine-tuning longitudinal controller)
![ezgif com-gif-maker](https://github.com/zcelil/vision-ROS/assets/57402408/143d1539-0792-45bf-b463-74e322e2caff)

