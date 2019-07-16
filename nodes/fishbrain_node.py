#!/usr/bin/env python

import roslib; roslib.load_manifest('fishgantry_ros')
import rospy
# from tetherfish_ros.msg import TetherFishMsg
import tf
import serial
import sys
import datetime
import rospkg
import nav_msgs
from geometry_msgs.msg import PoseStamped,Pose
from visualization_msgs.msg import Marker
import std_srvs.srv
from numpy import *

from PTW import *

class FishBrain():
    def __init__(self):

        # state 1: manual control
        # state 2: PTW antisocial
        # state 3: PTW social
        self.initialized = False
        self.manpath = MarkovChain()
        self.manpath.updateGeometry(.40,.15,.10)
        self.enabled = True
        self.home = False #make sure this gets implemented as a one-shot
        # self.state = 1

        self.timenow = rospy.Time.now()

        self.feedbacksub = rospy.Subscriber("/fishgantry/robotpose",PoseStamped,self.feedbackcallback)
        self.goalpose_pub = rospy.Publisher("/fishgantry/commandpose",PoseStamped,queue_size=1)
        self.tailpose_pub = rospy.Publisher("/fishgantry/tailpose",PoseStamped,queue_size=1)
        #initialize a command position
        self.command = PoseStamped()
        self.command.header.stamp = rospy.Time.now()
        self.command.pose.position.x = 0
        self.command.pose.position.y = 0
        self.command.pose.position.z = 0
        self.command.pose.orientation.z = 0
        self.command.pose.orientation.x = 0
        self.tailcommand = 0;
        self.rollcommand,self.pitchcommand,self.yawcommand=0,0,0

        self.br = tf.TransformBroadcaster()

        rospy.Service('set_manual_control',std_srvs.srv.Trigger,self.set_manual)
        rospy.Service('set_antisocial',std_srvs.srv.Trigger,self.set_antisocial)
        rospy.Service('set_social',std_srvs.srv.Trigger,self.set_social)

        #state 1 is manual, state 2 is antisocial, state 3 is social
        self.state = 2
        self.pose = PoseStamped()

        self.pose.header.stamp = rospy.Time.now()
        #initialize command
        self.pose.pose.position.x = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = 0
        self.pose.pose.orientation.x = 0
        self.pose.pose.orientation.y = 0
        self.pose.pose.orientation.z = 0
        self.pose.pose.orientation.w = 1

        

        self.dt = 0.01
        #main loop runs on a timer, which ensures that we get timely updates from the gantry arduino
        rospy.Timer(rospy.Duration(self.dt),self.loop,oneshot=False) #timer callback (math) allows filter to run at constant time
        rospy.Timer(rospy.Duration(.1),self.slowloop,oneshot=False)
        #subscribers (inputs)
        #construct the file name for our text output file
        rospack = rospkg.RosPack()
        # get the file path for rospy_tutorials
        self.package_path=rospack.get_path('fishgantry_ros')


        
        

    def feedbackcallback(self,data):
        if not self.initialized:
            # self.manpath.x = data.pose.position.x
            # self.manpath.y = data.pose.position.y
            # self.manpath.z = data.pose.position.z
            # r,p,a = tf.transformations.euler_from_quaternion([data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w])
            # self.manpath.pitchnow = p
            # self.manpath.psi = a
            self.initialized = True;

    def loop(self,data):
        if self.initialized:
            if self.state == 2:
                
                if self.enabled:

                    self.pose.pose.position.x,self.pose.pose.position.y,self.pose.pose.position.z,pitch,yaw,tail = self.manpath.drivePersistentFish(self.dt)
                    quat = tf.transformations.quaternion_from_euler(0,pitch,yaw)
                    self.pose.pose.orientation.x,self.pose.pose.orientation.y,self.pose.pose.orientation.z,self.pose.pose.orientation.w = quat
                    self.pose.header.stamp = rospy.Time.now()
                    self.goalpose_pub.publish(self.pose)
                    #rospy.logwarn("trying to publish pose from antisocial")
            else:
                pass
            
        else:
            self.goalpose_pub.publish(self.pose)
            rospy.logwarn("not initialized yet. publishing to get response")


    def slowloop(self,data):
        pass

    def set_manual(self,data):
        self.state = 1

    def set_antisocial(self,data):
        self.state = 2

    def set_social(self,data):
        self.state = 3


    #main function
def main(args):
  rospy.init_node('fishbrain_node', anonymous=True)
  fishbrain = FishBrain()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"

if __name__ == '__main__':
    main(sys.argv) 