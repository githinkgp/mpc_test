# Gazebo bot state Listener Class
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates

# begin class def
class StateListener:
    def __init__(self,topic):
       
        # init data
        self.topic = topic
        self.tStamp = 0.0
        self.State = [[],[]]

        # subscriber to the RPLIDAR
        #print 'initializing RPLIDAR subscribers...'
        self.subRplidar = rospy.Subscriber(topic,ModelStates,self.callback)
        
    # end of init

    # callback to LaserScan
    def callback(self,msg):
        self.State = [[],[]]
        names=msg.name
        pose=msg.pose
        twist=msg.twist
        idx=0
        for n in names:
            if n=="mybot":
                self.State[0].append(pose[idx])
                self.State[1].append(twist[idx])
            idx+=1
    
    # end callback

# end class def

