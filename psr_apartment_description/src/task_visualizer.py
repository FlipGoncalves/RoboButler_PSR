#!/usr/bin/python3

import os
import rospy
import rospkg
import cv2
import actionlib
import tf
import threading
from threading import Event
from functools import partial
from visualization_msgs.msg import Marker
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatus


### Correr o node como rosrun psr_apartment_description task_visualizer.py _object:=(class name) ###

pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)


#lista = []

def parse_classes_file(path):
    classes = []
    with open(path, "r") as f:
        for line in f:
            line = line.replace("\n", "")
            classes.append(line)
    return classes


def marker(text):
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.type = marker.TEXT_VIEW_FACING
    marker.scale.z = 0.35
    marker.color.a = 1.0
    marker.color.r = 0
    marker.color.g = 0
    marker.color.b = 0
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 1.4
    marker.pose.orientation.w = 1.0
    marker.text = text
    marker.lifetime = rospy.Duration(2)
    
    return marker

def pose_callback(pose_msg, actual_pose):
    actual_pose[0] = pose_msg.pose.pose
    


def image_callback(data, rospack, menu_msg, photo_taken):
    bridge = CvBridge()
    
    # Convert the ROS image message to a OpenCV image
    image = bridge.imgmsg_to_cv2(data, "bgr8")
    #cv2.imshow('image', image)
    
   
    # Take photo
    if menu_msg[0] == "take_photo" and not photo_taken[0]:
        photo_taken[0] = True 
        photo_num = 1
        text = marker("Taking a photo")
        pub.publish(text)

        while True:
            photo_path = rospack.get_path('psr_apartment_description') + '/photos/Foto_' + str(photo_num) + '.jpg'
            if not os.path.isfile(photo_path):
                break
            photo_num += 1
       
        cv2.imwrite(photo_path, image)

def send_goal_thread(client, goal):
    client.send_goal(goal)
    client.wait_for_result()



def callback_yolo(data, classes, id_list, places, menu_msg, goal_reached, moving_to_goal, look_for_obj, actual_pose, rot_over, obj_found):
    client_2 = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client_2.wait_for_server()
    msg = menu_msg[0]
    id_list =[]
    for detection in data.detections:
        # Get the id of each object
        object = detection.results[0].id
        id_list.append(object)
    print(id_list)  
    # Task 1 - Move somewhere
    if msg in places.keys() and not goal_reached[0] and not moving_to_goal[0]:
        moving_to_goal[0] = True
        send_goal(places, msg, goal_reached, moving_to_goal)
    elif goal_reached[0]:
        text = 'I am in the ' + msg  
        text_show = marker(text)
        pub.publish(text_show)
    
    # Task 2 - Look for an object
    if msg in classes: 

        # Define the goal
        for i in range(8):
            goal = Pose()
            goal.position = actual_pose[0].position
            goal.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, i * 45))
       
            rot_goal = MoveBaseGoal()
            rot_goal.target_pose.header.frame_id = "map"
            rot_goal.target_pose.pose = goal
                
            # Send the goal
            thread = threading.Thread(target=send_goal_thread, args=(client_2, rot_goal))
            thread.start()
            # text = 'Looking for ' + str(msg)
            # text_show = marker(text)
            # pub.publish(text_show)
            #print(client_2.get_state())
            # Check for new object detections
            rate = rospy.Rate(10.0000)

           
      
        rospy.on_shutdown(client_2.cancel_all_goals)
   
       
       
       
       
        # event = threading.Event()
        # thread = threading.Thread(target=rot_goal, args=(msg, rot_over, look_for_obj, actual_pose, classes))
        # thread.start()
        # rot_goal(msg, rot_over, look_for_obj, actual_pose, classes)
        # if classes.index(msg) in id_list:
        # # look_for_obj[0] = False
        #     print("found it!")
        #     # look_for_obj[0] = False 
        #     event.set()

        # text_show = marker('Found it!')
        # pub.publish(text_show)
         
    # elif rot_over[0]:
    #     text_show = marker('There is no ' + msg)
    #     pub.publish(text_show)

      
    
    
def callback_menu(data, menu_msg, photo_taken, goal_reached, moving_to_goal, look_for_obj, rot_over, obj_found):
    menu_msg[0] = data.data
    photo_taken[0] = False
    goal_reached[0] = False
    moving_to_goal[0] = False
    look_for_obj[0] = False
    rot_over[0] = False
    obj_found[0] = False
  
        # if classes.index(data.data) in id_list:
        #     marker.text = 'Found ' + str(id_list.count(classes.index(data.data))) + ' ' + data.data + '!'
        # else:
        #     marker.text = "Searching for " + 

def send_goal(places, target_name, goal_reached, moving_to_goal):
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()
    
    # Define the goal
    target = places[target_name]
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose = target['pose']

    # Send the goal
    if moving_to_goal[0] == True:
        client.send_goal(goal)
        text = 'Going to ' + str(target_name)
        while True:
            text_show = marker(text)
            pub.publish(text_show)

            if client.get_state() == GoalStatus.SUCCEEDED:
                goal_reached[0] = True
                break
            elif moving_to_goal[0] == False:
                client.cancel_all_goals() 
                break   
    else:
        client.cancel_all_goals()    
    
    # Wait for the result
    client.wait_for_result()
    rospy.on_shutdown(client.cancel_all_goals)
   

def rot_goal(target_name, rot_over, look_for_obj, actual_pose, classes):
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()
       
    
   
    # Define the goal
    for i in range(8):

        goal = Pose()
        goal.position = actual_pose[0].position
        goal.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, i * 45))

        rot_goal = MoveBaseGoal()
        rot_goal.target_pose.header.frame_id = "map"
        rot_goal.target_pose.pose = goal
         
        if classes.index(target_name) in id_list:
            client.cancel_all_goals()    
            break
            
        # Send the goal
        if look_for_obj[0] == True:
            client.send_goal(rot_goal)
            text = 'Looking for ' + str(target_name)
            
            text_show = marker(text)
            pub.publish(text_show)
            

            client.wait_for_result()

        else:
            client.cancel_all_goals()    
            break
  
    # Wait for the result
    client.wait_for_result()
    rospy.on_shutdown(client.cancel_all_goals)

        


def stop_publishing(): 
    pub.unregister()
    

def main():
    rospy.init_node('missions', anonymous=True)
    rospack = rospkg.RosPack()
    path = rospack.get_path('robutler_perception')
    classes = parse_classes_file(path + "/dataset/coco80.txt")
    id_list = []
    menu_msg = [None]
    actual_pose = [None]
    goal_reached = [False]
    photo_taken = [False]
    moving_to_goal = [False]
    look_for_obj = [False]
    rot_over = [False]
    obj_found = [False]
   
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


    yolo_sub = rospy.Subscriber("yolov7", Detection2DArray, partial(callback_yolo, classes=classes, id_list=id_list, places=places, menu_msg=menu_msg, goal_reached=goal_reached, moving_to_goal=moving_to_goal, look_for_obj=look_for_obj, actual_pose=actual_pose, rot_over=rot_over, obj_found=obj_found))
    image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, partial(image_callback, rospack=rospack, menu_msg= menu_msg, photo_taken=photo_taken))
    menu_sub = rospy.Subscriber('message', String, partial(callback_menu, menu_msg=menu_msg, photo_taken=photo_taken, goal_reached=goal_reached, moving_to_goal=moving_to_goal, look_for_obj=look_for_obj, rot_over=rot_over, obj_found=obj_found))
    pose_sub = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, partial(pose_callback, actual_pose=actual_pose))


    rospy.on_shutdown(stop_publishing) # when node shutdown calls the stop_publishing function
    rospy.spin()

   


if __name__ == '__main__':
    main()
    

   
    
#---------------------------------Missões--------------------------------------#

# Missão1: Verificar se numa divisão existe um objeto
# Missão2: Verificar se algum objeto não pertence a uma divisão
# Missão3: Tirar uma foto a um objeto/divisão
# Missão4: Verificar o numero de um objeto numa divisão 
# Missão5: Verificar se na casa existe um objeto
# Missão6: Verificar o numero de um objeto na casa
