#!/usr/bin/env python2
import cv2
import pyyolo
import rospy
import sys
import rospkg
import time

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from mm_messages.msg import object_detection

def callback(data,topic):
    global bridge
    global detector
    global detection

    prev_time = time.time()
    try:
        frame = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
    except CvBridgeError as e:
        print(e)
    dets = detector.detect(frame, rgb=False)
    for i, det in enumerate(dets):
        detection.min_x, detection.min_y, detection.max_x, detection.max_y = det.to_xyxy()
        detection.probability = det.prob
        detection.class_ = det.name
        detection.source = topic
        detection_pub.publish(detection)
        xmin, ymin, xmax, ymax = det.to_xyxy()
        
        cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 0, 255))
        cv2.putText(frame,
                    str(det.name) + " " + str(round(det.prob,2)),
                    (xmin,ymin),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    [0,0,255],
                    1)
        
    cv2.imshow('cvwindow', frame)
    cv2.waitKey(1)  
    stream_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
    rospy.loginfo(1/(time.time()-prev_time))

def main():
    global detector
    global bridge
    global detection
    global detection_pub
    global stream_pub
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('mm_object_detection')
    rospy.init_node('object_detector')
    detector = pyyolo.YOLO(pkg_path+"/src/cfg/yolov4.cfg",
                           pkg_path+"/src/yolov4.weights",
                           pkg_path+"/src/cfg/coco.data",
                           detection_threshold = 0.5,
                           hier_threshold = 0.5,
                           nms_threshold = 0.45)
    bridge = CvBridge()
    detection = object_detection()

    used_camera = "camera/data"
    image_sub = rospy.Subscriber(used_camera,Image,callback = callback, callback_args= used_camera)
    detection_pub = rospy.Publisher("object_detection/detection",object_detection,queue_size=0)
    stream_pub = rospy.Publisher("detection/output",Image,queue_size=0)


    rospy.spin()

if __name__ == '__main__':
    main()