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
    self.state = 1

    self.timenow = rospy.Time.now()
    self.port = rospy.get_param('~port','/dev/ttyACM0')
    self.baud = rospy.get_param('~baud',115200)
    self.ser = serial.Serial(self.port, self.baud,timeout=1) #this initializes the serial object
    self.pub= rospy.Publisher("fishgantry/robotpose",PoseStamped,queue_size=1)
    self.staticpubcmd = rospy.Publisher("/fishgantry/robotcmd_static",Marker,queue_size=1)
    self.ypubcmd = rospy.Publisher("/fishgantry/robotcmd_y",Marker,queue_size=1)
    self.xpubcmd = rospy.Publisher("/fishgantry/robotcmd_x",Marker,queue_size=1)
    self.yawpubcmd = rospy.Publisher("/fishgantry/robotcmd_yaw",Marker,queue_size=1)
    self.zpubcmd = rospy.Publisher("/fishgantry/robotcmd_z",Marker,queue_size=1)
    self.pitchpubcmd = rospy.Publisher("/fishgantry/robotcmd_pitch",Marker,queue_size=1)
    self.goalpose_sub = rospy.Subscriber("/fishgantry/commandpose",PoseStamped,self.commandCallback)
    self.tailpose_sub = rospy.Subscriber("/fishgantry/tailpose",PoseStamped,self.tailCallback)
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

    #initialize command
    self.pose.position.x = 0
    self.pose.position.y = 0
    self.pose.position.z = 0
    self.pose.orientation.x = 0
    self.pose.orientation.y = 0
    self.pose.orientation.z = 0
    self.pose.orientation.w = 1

    self.enabled = True
    self.home = False #make sure this gets implemented as a one-shot

    #main loop runs on a timer, which ensures that we get timely updates from the gantry arduino
    rospy.Timer(rospy.Duration(.01),self.loop,oneshot=False) #timer callback (math) allows filter to run at constant time
    rospy.Timer(rospy.Duration(.1),self.slowloop,oneshot=False)
    #subscribers (inputs)
    #construct the file name for our text output file
    rospack = rospkg.RosPack()
    # get the file path for rospy_tutorials
    self.package_path=rospack.get_path('fishgantry_ros')

def loop(self,data):
    pass

def slowloop(self,data):
    pass

def set_manual(self,data):
    self.state = 1

def set_antisocial(self,data):
    self.state = 2

def set_socialt(self,data):
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