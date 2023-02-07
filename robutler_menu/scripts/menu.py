#!/usr/bin/env python

"""
Copyright (c) 2011, Willow Garage, Inc.
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Willow Garage, Inc. nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

import rospy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from std_msgs.msg import String
from functools import partial


server = None

menu_handler = MenuHandler()


locations = ["living_room", "bedroom", "kitchen", "gym", "office", "outside", "dining_area"]

objects = ['person', "laptop", "chair", 'bottle', 'bed', 'clock', 'book', 'refrigerator', "stop_sign"]

missions = [
    "Look for bottle in dining_area",
    "Look for book in bedroom",
    "Look for stop_sign outside",
    "Look for person in gym",
    "Count chair in the dining_area",
    "Look for laptop in office",
    "Take photo of office",
    "Take photo of clock in living_room"
]


def navigationCb(location, feedback):
    handle = feedback.menu_entry_id
    rospy.loginfo(f"going to {location} (handle ID {handle})")
    #server.applyChanges()
    pub.publish(location)

def photoCb(feedback):
    rospy.loginfo("taking a photo")
    #server.applyChanges()
    pub.publish("take_photo")

def detectCb(obj, feedback):
    handle = feedback.menu_entry_id
    rospy.loginfo(f"looking for {obj}")
    #server.applyChanges()
    pub.publish(obj)


def countCb(obj, feedback):
    handle = feedback.menu_entry_id
    rospy.loginfo(f"counting {obj}")
    #server.applyChanges()
    pub.publish('count ' + obj)

def missionCb(mission, feedback):
    handle = feedback.menu_entry_id
    rospy.loginfo("Performing mission: " + mission)
    #server.applyChanges()
    pub.publish(mission)

def makeBall( msg ):
    marker = Marker()

    marker.type = Marker.SPHERE
    marker.scale.x = msg.scale
    marker.scale.y = msg.scale
    marker.scale.z = msg.scale
    marker.color.r = 0.9
    marker.color.g = 0.6
    marker.color.b = 0.5
    marker.color.a = 1.0

    return marker

def makeMenuMarker( name ):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.pose.position.z = 1.15
    int_marker.scale = 0.2
    int_marker.name = name

    control = InteractiveMarkerControl()
    control.interaction_mode = InteractiveMarkerControl.BUTTON
    control.always_visible = True

    control.markers.append(makeBall(int_marker))
    int_marker.controls.append(control)

    server.insert(int_marker)


def initMenu():
    # Navigation menu
    go_to_handle = menu_handler.insert("Go to...")
    for loc in locations:
        menu_handler.insert(loc, parent=go_to_handle, callback=partial(navigationCb, loc))

    # Visualization menu
    photo_handle = menu_handler.insert("Take photo", callback=photoCb)
    detect_handle = menu_handler.insert("Look for...")
    for obj in objects:
        menu_handler.insert(obj, parent=detect_handle, callback=partial(detectCb, obj))
    
    # Counting menu
    count_handle = menu_handler.insert("Count number of...")
    for obj in objects:
        menu_handler.insert(obj, parent=count_handle, callback=partial(countCb, obj))

    # Mission menu
    mission_handle = menu_handler.insert("Missions")
    for mission in missions:
        menu_handler.insert(mission, parent=mission_handle, callback=partial(missionCb, mission))


if __name__=="__main__":
    rospy.init_node("menu")
    
    server = InteractiveMarkerServer("menu")
    pub = rospy.Publisher('message', String, queue_size=10)
    initMenu()
    makeMenuMarker("marker")

    menu_handler.apply(server, "marker")
    server.applyChanges()

    rospy.spin()
