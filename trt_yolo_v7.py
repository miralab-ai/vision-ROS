#!/usr/bin/env python

import os
import time

import cv2
import pycuda.autoinit  # For initializing CUDA driver
import pycuda.driver as cuda
from torch import int32

from utils.yolo_classes import get_cls_dict
from utils.display import open_window, set_display, show_fps
from utils.visualization import BBoxVisualization
from utils.yolo_with_plugins import TrtYOLO

import rospy
import rospkg
from yolact_ros_msgs.msg import area_and_center
#from vision_msgs.msg import BoundingBox2D
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped


class yolov4(object):
    def __init__(self):
        """ Constructor """
        self.default = area_and_center()
        self.default.area   = 0             ## default value for longitudinal control ##    ## this value will be defined later according to field test ##
        self.default.hoz_cc = 320           ## default value for lateral control      ##    ## 640*480 frame
        self.default.ver_cc = 240           ## default value for altitude control     ##    ## 640*480 frame
        self.default.header.frame_id = "trt_yolo"
        self.default.header.stamp    = rospy.Time()
        self.bridge = CvBridge()
        self.init_params()
        self.init_yolo()
        self.cuda_ctx = cuda.Device(0).make_context()
        self.trt_yolo = TrtYOLO(
            (self.model_path + self.model), (self.h, self.w), self.category_num)

    def __del__(self):
        """ Destructor """

        self.cuda_ctx.pop()
        del self.trt_yolo
        del self.cuda_ctx

    def clean_up(self):
        """ Backup destructor: Release cuda memory """

        if self.trt_yolo is not None:
            self.cuda_ctx.pop()
            del self.trt_yolo
            del self.cuda_ctx

    def init_params(self):
        """ Initializes ros parameters """
        
        rospack = rospkg.RosPack()
        package_path = rospack.get_path("yolov4_trt_ros")
        self.video_topic = rospy.get_param("/video_topic", "/usb_cam/image_raw")
        self.model = rospy.get_param("/model", "yolov7")
        self.model_path = rospy.get_param(
            "/model_path", package_path + "/yolo/")
        self.category_num = rospy.get_param("/category_number", 1)
        self.input_shape = rospy.get_param("/input_shape", "288")
        self.conf_th = rospy.get_param("/confidence_threshold", 0.3)
        self.show_img = rospy.get_param("/show_image", True)
        self.image_sub = rospy.Subscriber(
            self.video_topic, Image, self.img_callback, queue_size=1, buff_size=640*480*3)
        self.detection_pub = rospy.Publisher(
            "/area_and_center", area_and_center, queue_size=1)
        self.overlay_pub = rospy.Publisher(
            "/camera_topic", Image, queue_size=1)
        #self.pose_pub = rospy.Publisher("/mavros/local_position/pose", PoseStamped, queue_size=1)

    def init_yolo(self):
        """ Initialises yolo parameters required for trt engine """

        if self.model.find('-') == -1:
            self.model = self.model + "-" + self.input_shape
            
        yolo_dim = self.model.split('-')[-1]

        if 'x' in yolo_dim:
            dim_split = yolo_dim.split('x')
            if len(dim_split) != 2:
                raise SystemExit('ERROR: bad yolo_dim (%s)!' % yolo_dim)
            self.w, self.h = int(dim_split[0]), int(dim_split[1])
        else:
            self.h = self.w = int(yolo_dim)
        if self.h % 32 != 0 or self.w % 32 != 0:
            raise SystemExit('ERROR: bad yolo_dim (%s)!' % yolo_dim)

        cls_dict = get_cls_dict(self.category_num)
        self.vis = BBoxVisualization(cls_dict)


    def img_callback(self, ros_img):
        """Continuously capture images from camera and do object detection """

        tic = time.time()

        # converts from ros_img to cv_img for processing
        try:
            cv_img = self.bridge.imgmsg_to_cv2(
                ros_img, desired_encoding="bgr8")
            rospy.logdebug("ROS Image converted for processing")
        except CvBridgeError as e:
            rospy.loginfo("Failed to convert image %s", str(e))

        if cv_img is not None:
            boxes, confs, clss = self.trt_yolo.detect(cv_img, self.conf_th)

            cv_img = self.vis.draw_bboxes(cv_img, boxes, confs, clss)
            toc = time.time()
            fps = 1.0 / (toc - tic)

            self.publisher(boxes, confs, clss)

            if self.show_img:
                cv_img = show_fps(cv_img, fps)
                cv2.imshow("YOLOv4 DETECTION RESULTS", cv_img)
                cv2.waitKey(1)

        # converts back to ros_img type for publishing
        try:
            overlay_img = self.bridge.cv2_to_imgmsg(
                cv_img, encoding="passthrough")
            rospy.logdebug("CV Image converted for publishing")
            self.overlay_pub.publish(overlay_img)
        except CvBridgeError as e:
            rospy.loginfo("Failed to convert image %s", str(e))

    def publisher(self, boxes, confs, clss):
        """ Publishes to detector_msgs

        Parameters:
        boxes (List(List(int))) : Bounding boxes of all objects
        confs (List(double))	: Probability scores of all objects
        clss  (List(int))	: Class ID of all classes
        """
        area_and_center_msg = area_and_center()
        #area_and_center_msg.header.stamp = rospy.Time()
        area_and_center_msg.header.frame_id = "trt_yolo"        
        boxes_num = len(boxes)
        #setpoint = PoseStamped()
        #detection2d.header.stamp = rospy.Time.now()
        #detection2d.header.frame_id = "camera" # change accordingly
        
        for i in range(boxes_num):
            # boxes : xmin, ymin, xmax, ymax
            #detection.header.stamp = rospy.Time.now()
                #detection.header.frame_id = "camera" # change accordingly
                #detection.id = int(clss[i])
                #detection.probability = confs[i]
                #print(detection.id)
                if int(clss[i]) == 0:

                    area_and_center_msg.hoz_cc = int(boxes[i][0] + (boxes[i][2] - boxes[i][0])/2)
                    area_and_center_msg.ver_cc = int(boxes[i][1] + (boxes[i][3] - boxes[i][1])/2)
                    area_and_center_msg.area   = int(abs(boxes[i][0] - boxes[i][2]) * abs(boxes[i][1] - boxes[i][3]))


                    self.default = area_and_center_msg
                    self.detection_pub.publish(area_and_center_msg)

                 
                #detection.center.x = boxes[i][0] + (boxes[i][2] - boxes[i][0])/2
                #detection.center.y = boxes[i][1] + (boxes[i][3] - boxes[i][1])/2
                #detection.center.theta = 0.0  # change if required

                #detection.size_x = abs(boxes[i][0] - boxes[i][2])
                #detection.size_y = abs(boxes[i][1] - boxes[i][3])
               
                #detection.xmin = boxes[i][0]
                #detection.ymin = boxes[i][1]
                #detection.xmax = boxes[i][2]
                #detection.ymax = boxes[i][3]
                

                #setpoint.pose.position.x = float(boxes[0][0] + (boxes[0][2] - boxes[0][0])/2)
                #setpoint.pose.position.y = float(boxes[0][1] + (boxes[0][3] - boxes[0][1])/2)
                #setpoint.pose.position.z = -10.0
                   
                #detection2d.bounding_boxes.append(detection)


        if boxes_num == 0:        
            self.detection_pub.publish(self.default)
        #self.pose_pub.publish(setpoint)


def main():
    yolo = yolov4()
    rospy.init_node('yolov7_trt_ros', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.on_shutdown(yolo.clean_up())
        print("Shutting down")


if __name__ == '__main__':
    main()
