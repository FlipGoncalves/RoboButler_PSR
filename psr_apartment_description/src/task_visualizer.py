#!/usr/bin/python3

import rospy
from functools import partial
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose

def parse_classes_file(path):
    classes = []
    with open(path, "r") as f:
        for line in f:
            line = line.replace("\n", "")
            classes.append(line)
    return classes

def callback_2(data):
    obj_id = None
    for detection in data.detections:
      
        # Get the id of each object
        object = detection.results[0].id
        return object

def callback_1(data, classes):
    marker = Marker()
    arg = rospy.get_param('~object', '0')
    obj_id = callback_2(data)
    marker.header.frame_id = "base_link"
    marker.type = marker.TEXT_VIEW_FACING
    marker.scale.z = 0.3
    marker.color.a = 1.0
    marker.color.r = 0
    marker.color.g = 0
    marker.color.b = 0
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 1.4
    marker.pose.orientation.w = 1.0

    if obj_id is not None and classes[obj_id] == arg:
        marker.text = "Found it!"
    else:
        marker.text = "Searching for " + arg

    pub.publish(marker)

def listener():
    rospy.init_node('listener', anonymous=True)
    classes = parse_classes_file("/home/rafael/catkin_ws/src/RoboButler_PSR/robutler_perception/dataset/coco80.txt")
    sub_1 = rospy.Subscriber("yolov7", Detection2DArray, partial(callback_1, classes=classes))
    rospy.spin()

if __name__ == '__main__':
    pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    listener()