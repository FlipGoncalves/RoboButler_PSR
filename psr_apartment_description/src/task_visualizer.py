#!/usr/bin/python3

import os
import rospy
import rospkg
import cv2
import actionlib
import tf
from functools import partial
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion


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
         

def callback_1(data, classes, id_list):
    marker = Marker()
    arg = rospy.get_param('~object', 'chair')
  
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
    
    if classes.index(arg) in id_list:
        marker.text = 'Found ' + str(id_list.count(classes.index(arg))) + ' ' + arg + '!'
    else:
        marker.text = "Searching for " + arg
    
    # Deletes the marker after node shutdown
    marker.lifetime = rospy.Duration(2)
    pub.publish(marker)

    

def send_goal(target_name):
   
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()
    
    # Places
    places = {
    "bedroom": {'pose': Pose(position=Point(x=-6, y=3.25, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1))},
    "gym": {'pose': Pose(position=Point(x=-3, y=3.25, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1))},
    "office": {'pose': Pose(position=Point(x=-0.34, y=3.25, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1))},
    "blue_object_division": {'pose': Pose(position=Point(x=1, y=1, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1))},
    "shoerack_division": {'pose': Pose(position=Point(x=1.78, y=-1.40, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1))},
    "outside": {'pose': Pose(position=Point(x=0.55, y=-3.9, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1))},
    "entrance": {'pose': Pose(position=Point(x=1.71, y=-2.57, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1))},
    "hall": {'pose': Pose(position=Point(x=0.45, y=-1.4, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1))},
    "living_room": {'pose': Pose(position=Point(x=-1.67, y=-3.9, z=0.), orientation=Quaternion(x=0,y=0,z=0,w=1))},
    "kitchen": {'pose': Pose(position=Point(x=-3.1, y=-0.9, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1))},
    "dining_area": {'pose': Pose(position=Point(x=-4.28, y=-2.1, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1))},
    "balcony": {'pose': Pose(position=Point(x=-7.25, y=-3.9, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1))},
    "main_corridor": {'pose': Pose(position=Point(x=-4, y=1.25, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1))}}
    
    # Define the goal
    target = places[target_name]
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose = target['pose']

    # Send the goal
    client.send_goal(goal)

    # Wait for the result
    client.wait_for_result()
    return places

def rot_goal(places, target_name):

    target = places[target_name]
    for i in range(8):
        rot_goal = Pose()

        rot_goal.position = target['pose'].position
        rot_goal.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, i * 45))

        send_goal(rot_goal)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospack = rospkg.RosPack()
    path = rospack.get_path('robutler_perception')
    classes = parse_classes_file(path + "/dataset/coco80.txt")
    id_list = []
    sub_1 = rospy.Subscriber("yolov7", Detection2DArray, partial(callback_1, classes=classes, id_list=id_list))
    image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, partial(image_callback, rospack=rospack))

    rospy.on_shutdown(stop_publishing) # when node shutdown calls the stop_publishing function
    rospy.spin()
    
def stop_publishing(): 
    pub.unregister()


if __name__ == '__main__':

    pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    listener()
    places = send_goal()
    
    rot_goal(places, )



#---------------------------------Missões--------------------------------------#

# Missão1: Verificar se numa divisão existe um objeto
# Missão2: Verificar se algum objeto não pertence a uma divisão
# Missão3: Tirar uma foto a um objeto/divisão
# Missão4: Verificar o numero de um objeto numa divisão 
# Missão5: Verificar se na casa existe um objeto
# Missão6: Verificar o numero de um objeto na casa
