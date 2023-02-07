#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

def move_claw():
    
    # right -> between 1 and 0 -» 1 open / 0 closed
    # left -> between -1 and 0 -» -1 open / 0 closed
    open = False
    if open:
        measure = 1
    else:
        measure = 0
    
    pub_right = rospy.Publisher('/robutler/pole_to_claw_right_second_joint_position_controller/command', Float64, queue_size=10)
    pub_left = rospy.Publisher('/robutler/pole_to_claw_left_second_joint_position_controller/command', Float64, queue_size=10)
    rospy.init_node('move_claw')
    rate = rospy.Rate(10) # 10hz
    count = 0
    while count != 5:
        pub_right.publish(float(measure))
        pub_left.publish(float(-measure))
        rate.sleep()
        count += 1
  
if __name__ == '__main__':
    try:
        move_claw()
    except rospy.ROSInterruptException:
        pass