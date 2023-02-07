#!/usr/bin/python3

# Import necessary modules
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

class TaskVisualizer:
    def __init__(self):
        self.rospack = rospkg.RosPack()
        self.path = self.rospack.get_path('robutler_perception')
        self.classes = self.parse_classes_file(self.path + "/dataset/coco80.txt")

        self.imdage_data = None
        self.mission_1 = [None]
        self.mission_2 = [None]
        self.mission_3 = [None]
        self.mission_4 = [None]
        self.menu_msg = [None]
        self.actual_pose = [None]
        self.goal_reached = [False]
        self.photo_taken = [False]
        self.moving_to_goal = [False]
        self.look_for_obj = [False]
        self.rot_over = [False]
        self.obj_found = [False]
        

        self.places = {
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
        "dining_area": {'pose': Pose(position=Point(x=-4.5, y=-2.5, z=0), orientation=Quaternion(x=0, y=0, z=-0.7, w=0.7))},
        "balcony": {'pose': Pose(position=Point(x=-7.25, y=-3.9, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1))},
        "main_corridor": {'pose': Pose(position=Point(x=-4, y=1.25, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1))}}

        yolo_sub = rospy.Subscriber("yolov7", Detection2DArray, self.callback_yolo)
        image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        menu_sub = rospy.Subscriber('message', String, self.callback_menu)
        pose_sub = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.pose_callback)
        self.pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
       
        rospy.on_shutdown(self.stop_publishing)
    
    def parse_classes_file(self, path):
        classes = []
        with open(path, "r") as f:
            for line in f:
                line = line.replace("\n", "")
                classes.append(line)
        return classes

    def marker(self, text):
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

    def pose_callback(self, pose_msg):
        self.actual_pose[0] = pose_msg.pose.pose
    

    def image_callback(self, data):
        bridge = CvBridge()
     
        # Convert the ROS image message to a OpenCV image
        self.image_data = bridge.imgmsg_to_cv2(data, "bgr8")
        # cv2.imshow('image', image)
        # print(self.id_list)
    
        # Take photo
        if self.menu_msg[0] == "take_photo" and not self.photo_taken[0]:
            self.photo_taken[0] = True 
            photo_num = 1
            text = self.marker("Taking a photo")
            self.pub.publish(text)

            while True:
                photo_path = self.rospack.get_path('psr_apartment_description') + '/photos/Foto_' + str(photo_num) + '.jpg'
                if not os.path.isfile(photo_path):
                    break
                photo_num += 1
        
            cv2.imwrite(photo_path, self.image_data)

        
        if self.menu_msg[0]:
            msg_words = self.menu_msg[0].split()
            
            # Mission 3 - Take photo of office
            if msg_words[0] == 'Take' and not self.goal_reached[0] and not self.moving_to_goal[0]:
                self.moving_to_goal[0] = True
                self.mission_3[0] = True
                thread_2 = threading.Thread(target=self.send_goal, args=(self.places, msg_words[3], self.goal_reached, self.moving_to_goal))
                thread_2.start()     
            if self.goal_reached[0] and self.mission_3[0] and not self.photo_taken[0]:
                rospy.sleep(2)  
                self.photo_taken[0] = True 
                photo_num = 1
                text = self.marker("Taking a photo")
                self.pub.publish(text)
                while True:
                    photo_path = self.rospack.get_path('psr_apartment_description') + '/photos/Foto_' + str(photo_num) + '.jpg'
                    if not os.path.isfile(photo_path):
                        break
                    photo_num += 1
        
                cv2.imwrite(photo_path, self.image_data)
         


    def callback_yolo(self, data):
  
        msg = self.menu_msg[0]
        id_list= []
    
        for detection in data.detections:
            # Get the id of each object
            object = detection.results[0].id
            id_list.append(object)
    
        # Task 1 - Move somewhere
        if msg in self.places.keys() and not self.goal_reached[0] and not self.moving_to_goal[0]:
            self.moving_to_goal[0] = True
            self.send_goal(self.places, msg, self.goal_reached, self.moving_to_goal)
        elif self.goal_reached[0] and not self.look_for_obj[0] and not self.mission_2[0] and not self.mission_3[0] and not self.mission_4[0]:
            text = 'I am in the ' + msg  
            text_show = self.marker(text)
            self.pub.publish(text_show)
        
        # Task 2 - Look for an object
        if msg in self.classes:
            if self.classes.index(msg) in id_list:
                self.obj_found[0]=True
                text_show = self.marker('Found it!')
                self.pub.publish(text_show)
            if not self.look_for_obj[0] and not self.obj_found[0]:
                self.look_for_obj[0] = True
                thread = threading.Thread(target=self.rot_goal, args=(msg, self.look_for_obj, self.actual_pose, self.obj_found, self.rot_over))
                thread.start()
            if self.rot_over[0]:
                text_show = self.marker('Did not find any ' + msg)
                self.pub.publish(text_show)

        # Task 3 - Count the number of an object
        if msg:
            msg_words_1 = msg.split()
            if msg_words_1[0] == 'count':
                if self.classes.index(msg_words_1[1]) in id_list:
                    counter = id_list.count(self.classes.index(msg_words_1[1]))
                    if counter > 1:
                        text_show = self.marker('There are ' + str(counter) + ' ' + msg_words_1[1])
                        self.pub.publish(text_show)
                    elif counter == 1:
                        text_show = self.marker('There is '  + str(1) + msg_words_1[1])
                        self.pub.publish(text_show)
                else:
                    text_show = self.marker('There is no ' + msg_words_1[1])
                    self.pub.publish(text_show)
        
            # Mission 1 - Looking for laptop in office
            if msg_words_1[0] == 'Look' and not self.goal_reached[0] and not self.moving_to_goal[0]:
                self.moving_to_goal[0] = True
                self.mission_1[0] = True
                self.send_goal(self.places, msg_words_1[4], self.goal_reached, self.moving_to_goal)
            if self.goal_reached[0] and self.mission_1[0]:
                if self.classes.index(msg_words_1[2]) in id_list:
                    self.obj_found[0]=True
                    text_show = self.marker('Found it!')
                    self.pub.publish(text_show)
                if not self.look_for_obj[0] and not self.obj_found[0]:
                    self.look_for_obj[0] = True
                    thread_1 = threading.Thread(target=self.rot_goal, args=(msg_words_1[2], self.look_for_obj, self.actual_pose, self.obj_found, self.rot_over))
                    thread_1.start()
                if self.rot_over[0]:
                    text_show = self.marker('Did not find any ' + msg_words_1[2])
                    self.pub.publish(text_show)
            
            # Mission 2 - Count the number of chairs in the dining room
            if msg_words_1[0] == 'Count' and not self.goal_reached[0] and not self.moving_to_goal[0]:
                self.moving_to_goal[0] = True
                self.mission_2[0] = True
                self.send_goal(self.places, msg_words_1[4], self.goal_reached, self.moving_to_goal)
            if self.goal_reached[0] and self.mission_2[0]:
                if self.classes.index(msg_words_1[1]) in id_list:
                    counter = id_list.count(self.classes.index(msg_words_1[1]))
                    if counter > 1:
                        text_show = self.marker('There are ' + str(counter) + ' ' + msg_words_1[1])
                        self.pub.publish(text_show)
                    elif counter == 1:
                        text_show = self.marker('There is '  + str(1) + msg_words_1[1])
                        self.pub.publish(text_show)
                else:
                    text_show = self.marker('There is no ' + msg_words_1[1])
                    self.pub.publish(text_show)

            # Missão 4 - Take photo of clock in living_room
            if msg_words_1[0] == 'Take' and not self.goal_reached[0] and not self.moving_to_goal[0]:
   
                self.moving_to_goal[0] = True
                self.mission_4[0] = True
                self.send_goal(self.places, msg_words_1[5], self.goal_reached, self.moving_to_goal)
                
            if self.goal_reached[0] and self.mission_4[0]:
                if self.classes.index(msg_words_1[3]) in id_list and not self.photo_taken[0]:
                    self.obj_found[0]=True
                  
                    rospy.sleep(4)  
                    self.photo_taken[0] = True 
                    photo_num = 1
                    text_show = self.marker('Found it! Taking photo')
                    self.pub.publish(text_show)
              
                    while True:
                        photo_path = self.rospack.get_path('psr_apartment_description') + '/photos/Foto_' + str(photo_num) + '.jpg'
                        if not os.path.isfile(photo_path):
                            break
                        photo_num += 1
        
                    cv2.imwrite(photo_path, self.image_data)
                if not self.look_for_obj[0] and not self.obj_found[0]:
                    self.look_for_obj[0] = True
                    thread_4 = threading.Thread(target=self.rot_goal, args=(msg_words_1[3], self.look_for_obj, self.actual_pose, self.obj_found, self.rot_over))
                    thread_4.start()
                if self.rot_over[0]:
                    text_show = self.marker('Did not find any ' + msg_words_1[3])
                    self.pub.publish(text_show)  

    def callback_menu(self, data):
        self.menu_msg[0] = data.data
        self.photo_taken[0] = False
        self.goal_reached[0] = False
        self.moving_to_goal[0] = False
        self.look_for_obj[0] = False
        self.rot_over[0] = False
        self.obj_found[0] = False
        self.mission_3[0] = False
        self.mission_4[0] = False
   
       
    def send_goal(self, places, target_name, goal_reached, moving_to_goal):
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
                text_show = self.marker(text)
                self.pub.publish(text_show)

                if client.get_state() == GoalStatus.SUCCEEDED:
                    goal_reached[0] = True
                    break
                elif moving_to_goal[0] == False:
                    client.cancel_all_goals() 
                    break   
        else:
            client.cancel_all_goals()    
        
        rospy.sleep(1)
        # Wait for the result
        client.wait_for_result()
        rospy.on_shutdown(client.cancel_all_goals)
    

    def rot_goal(self, target_name, look_for_obj, actual_pose, obj_found, rot_over):
        client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        client.wait_for_server()

        rospy.Rate(5)   
        # Define the goal
        n=0
        for i in range(8):

            goal = Pose()
            goal.position = actual_pose[0].position
            goal.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, i * 45))

            rot_goal = MoveBaseGoal()
            rot_goal.target_pose.header.frame_id = "map"
            rot_goal.target_pose.pose = goal

            # Send the goal
            if look_for_obj[0] == True:

                client.send_goal(rot_goal)
                text = 'Looking for ' + str(target_name)
                
                while True:
                    text_show = self.marker(text)
                    self.pub.publish(text_show)
                    if obj_found[0] == True:
                        client.cancel_all_goals() 
                        break

                    if client.get_state() == GoalStatus.SUCCEEDED:
                        break
    
                    if look_for_obj[0] == False:
                        client.cancel_all_goals() 
                        break  
            else:
                client.cancel_all_goals()    
                break
            n +=1
            # print(n)
            if n ==8 and not obj_found[0]:
                rot_over[0]=True 
        
        rospy.sleep(1)     
        # Wait for the result
        client.wait_for_result()
        rospy.on_shutdown(client.cancel_all_goals)


    def stop_publishing(self): 
        self.pub.unregister()

node = TaskVisualizer()
rospy.init_node("test_node")
rospy.spin()