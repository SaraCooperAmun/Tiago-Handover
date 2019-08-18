#By: Sara Cooper
#Writing data to rosbag file
#! /usr/bin/env python

import rosbag
from std_msgs.msg import Int32, String
from geometry_msgs.msg import Wrench
    
bag = rosbag.Bag('/home/sara/test.bag', 'a')
if(True):
       s = String()
       s.data = 'foo4'
   
       bag.write('chatter', s)
       data = Wrench()
       data.force.x = 12
       data.force.y = 10
       data.force.z = 12
       data.torque.x = 1
       data.torque.y = 6
       data.torque.z = 10      
       bag.write('/wrist_ft', data)
       bag.close()

