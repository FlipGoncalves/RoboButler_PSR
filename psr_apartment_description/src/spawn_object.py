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
            # "camera",
            # "cube",
            # "cup",
            # "donut",
            # "plant",
            # "pot_flower",
            "sphere",
            "trash_bin",
            "vase_glass",
            "football",
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
    "bedside_cabinet": {'pose': Pose(position=Point(x=-7.33, y=5.29, z=0.6), orientation=Quaternion(x=0,y=0,z=0,w=1)), 'room': 'bedroom'},
    "corner_cabinet": {'pose': Pose(position=Point(x=-7.04, y=2.85, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1)), 'room': 'bedroom'},
    "bed": {'pose': Pose(position=Point(x=-5.69, y=4.37, z=0.65), orientation=Quaternion(x=0,y=0,z=0,w=1)), 'room': 'bedroom'},
    
    "beside_cabinet": {'pose': Pose(position=Point(x=-0.88, y=4.97, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1)), 'room': 'office'},
    "top_chair_office": {'pose': Pose(position=Point(x=1.18, y=5.22, z=0.49), orientation=Quaternion(x=0,y=0,z=0,w=1)), 'room': 'office'},
    "top_desk": {'pose': Pose(position=Point(x=1.02, y=3.2, z=0.74), orientation=Quaternion(x=0,y=0,z=0,w=1)), 'room': 'office'},
    "under_desk": {'pose': Pose(position=Point(x=1.60, y=3.36, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1)), 'room': 'office'},
    
    "next_plant": {'pose': Pose(position=Point(x=-7.58, y=-0.15, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1)), 'room': 'balcony'},
    "corner_plant": {'pose': Pose(position=Point(x=-7.54, y=-5.14, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1)), 'room': 'balcony'},

    "top_chair": {'pose': Pose(position=Point(x=-6.2, y=-5.2, z=0.49), orientation=Quaternion(x=0,y=0,z=0,w=1)), 'room': 'dining_area'},
    "under_table": {'pose': Pose(position=Point(x=-4.9, y=-4.3, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1)), 'room': 'dining_area'},
    "ontop_table": {'pose': Pose(position=Point(x=-5.5, y=-3.6, z=1.01), orientation=Quaternion(x=0,y=0,z=0,w=1)), 'room': 'dining_area'},
    "ontop_coffee_table": {'pose': Pose(position=Point(x=-5.48, y=-2.2, z=0.76), orientation=Quaternion(x=0,y=0,z=0,w=1)), 'room': 'dining_area'}, #0.483 with the new table
    "behind_coffee_table": {'pose': Pose(position=Point(x=-6.43, y=-1.83, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1)), 'room': 'dining_area'},

    "corner_fridge": {'pose': Pose(position=Point(x=-3.25, y=0.08, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1)), 'room': 'kitchen'},
    "corner_atop": {'pose': Pose(position=Point(x=-3.56, y=-1.68, z=0.91), orientation=Quaternion(x=0,y=0,z=0,w=1)), 'room': 'kitchen'},
    "near_sink": {'pose': Pose(position=Point(x=-1.83, y=-1.68, z=0.91), orientation=Quaternion(x=0,y=0,z=0,w=1)), 'room': 'kitchen'},
    "front_person": {'pose': Pose(position=Point(x=-2.42, y=-0.45, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1)), 'room': 'kitchen'},
    
    "near_tv": {'pose': Pose(position=Point(x=-0.66, y=-4.7, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1)), 'room': 'living_room'},
    "front_tv": {'pose': Pose(position=Point(x=-0.7, y=-3.82, z=0.325), orientation=Quaternion(x=0,y=0,z=0,w=1)), 'room': 'living_room'},
    "behind_sofa": {'pose': Pose(position=Point(x=-3.32, y=-5.4, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1)), 'room': 'living_room'},
    "corner_sofa": {'pose': Pose(position=Point(x=-2.3, y=-5.4, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1)), 'room': 'living_room'},
    "atop_bookshelf": {'pose': Pose(position=Point(x=-1.63, y=-2.37, z=1.20), orientation=Quaternion(x=0,y=0,z=0,w=1)), 'room': 'living_room'},

    "atop_shoerack": {'pose': Pose(position=Point(x=1.87, y=-0.48, z=1.025), orientation=Quaternion(x=0,y=0,z=0,w=1)), 'room': 'shoerack_division'},
    "corner": {'pose': Pose(position=Point(x=1.23, y=-0.41, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1)), 'room': 'shoerack_division'},

    "corner_far_right": {'pose': Pose(position=Point(x=1.64, y=0.1, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1)), 'room': 'blueblock_division'},
    "corner_far_left": {'pose': Pose(position=Point(x=1.64, y=2, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1)), 'room': 'blueblock_division'},

    "top_cabinet": {'pose': Pose(position=Point(x=-1.84, y=3.72, z=0.39), orientation=Quaternion(x=0,y=0,z=0,w=1)), 'room': 'gym'},
    "between_equipement": {'pose': Pose(position=Point(x=-3.67, y=4.22, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1)), 'room': 'gym'},
    
}

# complex placements -> placements for all objects we want to spawn
complex_placements = {
    "corner": {'pose': Pose(position=Point(x=1.44, y=2.6, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1)), 'room': 'office'},
    "beside_fridge": {'pose': Pose(position=Point(x=-4.66, y=0.07, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1)), 'room': 'dining_area'},
    "near_wardrobe": {'pose': Pose(position=Point(x=-3.41, y=0.7, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1)), 'room': 'main_corridor'},
    "near_balcony": {'pose': Pose(position=Point(x=-6.4, y=1.88, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1)), 'room': 'main_corridor'},
    "corner_bed": {'pose': Pose(position=Point(x=-4.62, y=5.13, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1)), 'room': 'bedroom'},
    "entrance": {'pose': Pose(position=Point(x=1.71, y=-2.57, z=0), orientation=Quaternion(x=0,y=0,z=0,w=1)), 'room': 'outside'},
}

total_placements = len(list(basic_placements.keys())) + len(list(complex_placements.keys()))

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

print(spawned_models)

if total_placements == len(spawned_models):
    print("All places are already in use, please delete an object and continue...")
    exit(0)

# get param values
model_name = rospy.get_param('~object', "-1")
model_placement = rospy.get_param('~place', "-1")

temp = model_placement

while True:
    # variable to end while loop
    place_used = False
    
    # case its not specified the object we want to spawn 
    if model_name == "-1" or model_name == -1:
        prob = random.randint(1, 100)
        # 30% chance of getting a complex model instead of a basic model
        if prob > 30 or model_placement in list(basic_placements.keys()):
            model_name = random.choice(basic_models)
        else:
            model_name = random.choice(complex_models)
    else:
        if model_name not in basic_models and model_name not in complex_models:
            print("Object does not exist!!")
            print(model_placement, model_name)
            exit(0)

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
        
        if model_placement in list(basic_placements.keys()):
            model_placement = (model_placement, basic_placements[model_placement])
        elif model_placement in list(complex_placements.keys()):
            model_placement = (model_placement, complex_placements[model_placement])
        else:
            print("Place does not exist!!")
            print(model_placement, model_name)
            exit(0)
        
        for model in spawned_models:
            if "_"+model_placement[0]+"_" in model:
                print("Place already in use, please choose a different place")
                exit(0)
                
        break
            
    for model in spawned_models:
        if "_"+model_placement[0]+"_" in model:
            place_used = True
            break
        
    if not place_used:
        break
    
    model_placement = temp
    

if model_name in complex_models and model_placement[0] not in list(complex_placements.keys()):
    print("Place and Object do not correspond!!")
    print(model_placement[0], model_name)
    exit(0)

            
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

"""
MAIN DIVISIONS    
---------------------
office -0.34 3.25 0
dining_area -4.85 -2.1 0
main_corridor -4 1.25 0 
bedroom -6 3.25 0    
outside 0.55 -3.9 0
hall 0.45 -1.4 0
balcony -7.25 -3.9 0
kitchen -3.1 -0.9 0
living_room -1.67 -3.9 0
shoerack_division 1.78 -1.40 0
blueblock_division 1 1 0
gym -3 3.25 0
"""