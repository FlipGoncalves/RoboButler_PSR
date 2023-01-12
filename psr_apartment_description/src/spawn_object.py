#!/usr/bin/env python3

import random
import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion

""" Código do stor """
# # Add here mode poses
#
# model_names = ['sphere_v', 'cube']
#
# # Add here several models. All should be added to the robutler_description package
# model_name = random.choice(model_names)
# model_placement = random.choice(placements)


""" Código nosso """
rospy.init_node('insert_object',log_level=rospy.INFO)

# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()
package_path = rospack.get_path('psr_apartment_description') + '/description/models/'

rospy.wait_for_service('gazebo/spawn_sdf_model')
spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)

model_name = rospy.get_param('~object', "cube")
model_placement = rospy.get_param('~place', "bedside_cabinet")

f = open( package_path + model_name + '/model.sdf' ,'r')
sdff = f.read()

placements = {
                "bedside_cabinet": {'pose': Pose(position=Point(x=-7.33, y=5.29, z=0.58), orientation=Quaternion(x=0,y=0,z=0,w=1)), 'room': 'large_bedroom'},
                "bed": {'pose': Pose(position=Point(x=-5.69, y=4.37, z=0.6), orientation=Quaternion(x=0,y=0,z=0,w=1)), 'room': 'large_bedroom'}
            }

name = model_name + '_in_' + model_placement + '_of_' + placements[model_placement]['room']
spawn_model_prox(name, sdff, model_name, placements[model_placement]['pose'], "world")


# to run: rosrun psr_apartment_description spawn_object.py _place:=<place> _object:=<object>