#!/usr/bin/python3

import os
import rospy
import rospkg
import cv2
from functools import partial
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


### Correr o node como rosrun psr_apartment_description task_visualizer.py _object:=(class name) ###

def parse_classes_file(path):
    classes = []
    with open(path, "r") as f:
        for line in f:
            line = line.replace("\n", "")
            classes.append(line)
    return classes

def image_callback(data, rospack):
    bridge = CvBridge()
    
    # Convert the ROS image message to a OpenCV image
    
    # print(image)
    # photo_path = rospack.get_path('psr_apartment_description')
    #cap = cv2.VideoCapture('image')
   
    image = bridge.imgmsg_to_cv2(data, "bgr8")
    cv2.imshow('image', image)
    
    # Take photo
    if cv2.waitKey(1) == ord('p'):
        photo_num = 1
        while True:
            photo_path = rospack.get_path('psr_apartment_description') + '/photos/Foto_' + str(photo_num) + '.jpg'
            if not os.path.isfile(photo_path):
                break
            photo_num += 1
       
        cv2.imwrite(photo_path, image)
        
    
    # cv2.destroyAllWindows()    

def callback_1(data, classes, classes_list, id_list):
    marker = Marker()
    arg = rospy.get_param('~object', '0')
  
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
    id_list = []
    for detection in data.detections:
    
        # Get the id of each object
        object = detection.results[0].id
        id_list.append(object)  
        # print(id_list)   
        if object is not None and classes[object] not in classes_list:
            
            classes_list.append(classes[object])
            # print(classes_list)
        
        # elif object is not None and 
    
    if arg in classes_list:
        marker.text = 'Found ' + str(id_list.count(classes.index(arg))) + ' ' + arg + '!'
        classes_list[:] = []
    else:
        marker.text = "Searching for " + arg
    
    # Deletes the marker after node shutdown
    marker.lifetime = rospy.Duration(2)
    
    pub.publish(marker)

   
def listener():
    rospy.init_node('listener', anonymous=True)
    rospack = rospkg.RosPack()
    path = rospack.get_path('robutler_perception')
    classes = parse_classes_file(path + "/dataset/coco80.txt")
    classes_list = []
    id_list = []
    sub_1 = rospy.Subscriber("yolov7", Detection2DArray, partial(callback_1, classes=classes, classes_list=classes_list, id_list=id_list))
    image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, partial(image_callback, rospack=rospack))

    rospy.on_shutdown(stop_publishing) # when node shutdown calls the stop_publishing function
    rospy.spin()
    
def stop_publishing(): 
    pub.unregister()


if __name__ == '__main__':

    pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    listener()





