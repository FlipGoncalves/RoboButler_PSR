#!/usr/bin/env python3

import random
import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel, GetWorldProperties
from geometry_msgs.msg import Pose, Point, Quaternion

""" Código nosso """
# start rosnode
rospy.init_node('insert_object',log_level=rospy.INFO)

# basic models -> models that dont need much space and are easier to spawn in any place
basic_models = [
            "beer",
            "book",
            "bowl",
            "camera",
            "cube",
            "cup",
            "donut",
            "plant",
            "pot_flower",
            "sphere",
            "trash_bin",
            "vase_glass",
        ]

# complex models -> models bigger in size and harder to spawn inside the house
complex_models = [
            "human_male",
            "human_female",
            #"stop_light_post",             i dont think this is a good idea XD
            "stop_sign",
        ]

# basic placements -> placements for the basic objects we want to spawn
basic_placements = {
    "bedside_cabinet": {'pose': Pose(position=Point(x=-7.33, y=5.29, z=0.6), orientation=Quaternion(x=0,y=0,z=0,w=1)), 'room': 'large_bedroom'},
    "bed": {'pose': Pose(position=Point(x=-5.69, y=4.37, z=0.65), orientation=Quaternion(x=0,y=0,z=0,w=1)), 'room': 'large_bedroom'}
}

# complex placements -> placements for the complex objects we want to spawn
complex_placements = {
    "entrance": {'pose': Pose(position=Point(x=1.71, y=-2.57, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1)), 'room': 'outside'}
}

# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()
package_path = rospack.get_path('psr_apartment_description') + '/description/models/'
# get gazebo models for object spawning
rospy.wait_for_service('gazebo/spawn_sdf_model')
spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)

rospy.wait_for_service('gazebo/get_world_properties')
getProperties = rospy.ServiceProxy('gazebo/get_world_properties', GetWorldProperties)

all_models = getProperties.call().model_names
spawned_models = [model for model in all_models if "_in_" in model and "_of_" in model]
# print(spawned_models)

# get param values
model_name = rospy.get_param('~object', "-1")
model_placement = rospy.get_param('~place', "-1")

while True:
    # variable to end while loop
    place_used = False
    
    # case its not specified the object we want to spawn 
    if model_name == "-1" or model_name == -1:
        prob = random.randint(0, 100)
        # 30% chance of getting a complex model instead of a basic model
        if prob > 30:
            model_name = random.choice(basic_models)
        else:
            model_name = random.choice(complex_models)

    # case its not specified the place we want to spawn our object in 
    if model_placement == "-1" or model_placement == -1:
        # case we have a basic model
        if model_name in basic_models:
            place = random.choice(list(basic_placements.keys()))
            model_placement = (place, basic_placements[place])
        # case we have a complex model
        else:
            place = random.choice(list(complex_placements.keys()))
            model_placement = (place, complex_placements[place])
    else:
        for model in spawned_models:
            if model_placement[0] in model:
                print("Place already in use, please choose a different place")
                exit(0)
            
    for model in spawned_models:
        if model_placement[0] in model:
            place_used = True
            break
        
    if not place_used:
        break
            
# name the object
name = model_name + '_in_' + model_placement[0] + '_of_' + model_placement[1]['room']

print(name)

# get the specific model
f = open( package_path + model_name + '/model.sdf' ,'r')
sdff = f.read()

# spawn the object
spawn_model_prox(name, sdff, model_name, model_placement[1]['pose'], "world")



"""
to run: 
    rosrun psr_apartment_description spawn_object.py _place:=<place> _object:=<object>
            --or--
    rosrun psr_apartment_description spawn_object.py _place:=<place> _object:=-1
            --or--
    rosrun psr_apartment_description spawn_object.py _place:=-1 _object:=<object>
            --or--
    rosrun psr_apartment_description spawn_object.py _place:=-1 _object:=-1
"""