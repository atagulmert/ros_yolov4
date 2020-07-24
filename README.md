# ros_yolov4
an implementation of Yolov4 algorithm to ROS using darknet framework and a python wrapper (PyYOLO)

# How to use

In detection.py configure your cfg, data and weight file locations on those lines; 

```
detector = pyyolo.YOLO(pkg_path+"/src/cfg/yolov4.cfg", # cfg file location
                           pkg_path+"/src/yolov4.weights", # weight file location
                           pkg_path+"/src/cfg/coco.data", # data file location
                           detection_threshold = 0.5,
                           hier_threshold = 0.5,
                           nms_threshold = 0.45)
```

Then add detection.py as node to your launch file as seen in example here. Don't forget to remap "camera/data" to source Image topic and detection/output to desired output Image topic.
All detections will be also published on object_detection/detections in a custom message format I named as object_detection. You may need to create and generate that message type before you publish that.
Or just comment out those in detection.py;

```
#line 25 - 29
detection.min_x, detection.min_y, detection.max_x, detection.max_y = det.to_xyxy()
detection.probability = det.prob
detection.class_ = det.name
detection.source = topic
detection_pub.publish(detection)

#line 50
global detection_pub

#line 66
detection_pub = rospy.Publisher("object_detection/detection",object_detection,queue_size=0)
```

object_detection.msg
```
int16 min_x
int16 max_x
int16 min_y
int16 max_y
string class
float32 probability
string source
```
