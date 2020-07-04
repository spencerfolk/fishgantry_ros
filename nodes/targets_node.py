#!/usr/bin/env python

import roslib; roslib.load_manifest('fishgantry_ros')
import rospy
from std_msgs.msg import * #import all of the standard message types
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped,Pose
from numpy import *
import time
import serial
import tf
from hunting_fishbrain.hunting_fishbrain.TwoTargets import TwoTargets
from hunting_fishbrain.hunting_fishbrain.HybridFishBrain import FishState


#this node subscribes to a float and a string. The float represents the input to a first order system. The string represents a state.

class TargetsNode():
  def __init__(self):
    self.timenow = rospy.Time.now()#in case you need this
    self.listener = tf.TransformListener()
    #set up your publishers with appropriate topic types

    self.conditionpub = rospy.Publisher("/experimental_condition",String,queue_size=1)
    self.statepub = rospy.Publisher("/target_is_out",String,queue_size=1)
    self.targetposepub = rospy.Publisher("/targetpose",Pose,queue_size=1)
    self.targetMarkerpub = rospy.Publisher("/target_marker",Marker,queue_size=1)

    self.xoffset,self.yoffset,self.zoffset = -.15,-.05,0

    self.port = rospy.get_param('~port','/dev/ttyACM0')
    self.baud = rospy.get_param('~baud',115200)
    self.ser = serial.Serial(self.port, self.baud,timeout=1) #this initializes the serial object

    #set up your subscribers
    self.sub1 = rospy.Subscriber("/robotshot",Bool,self.sub1Callback)
    #initialize any variables that the class "owns. these will be available in any function in the class.
    #(self,ITI_mean,ITI_random,Trial_mean,Trial_random,tleftPose,trightPose)
    ITI_mean,ITI_random,Trial_mean,Trial_random = 5,0,25,0
    rTarg = FishState(.35,.15,.15,0,0) #the target has no inherent pitch or yaw requirement
    lTarg = FishState(.05,.15,.15,0,0) 
    self.targets = TwoTargets(ITI_mean,ITI_random,Trial_mean,Trial_random,lTarg,rTarg)

    self.dt = 0.1
    #set up timed loop to run like an arduino's "void loop" at a particular rate (100Hz)
    rospy.Timer(rospy.Duration(self.dt),self.loop,oneshot=False) 

    # now some stuff for the arduino
    self.outPosition = 90
    self.inPosition = 0

    #transform broadcaster
    self.br = tf.TransformBroadcaster()


  def sub1Callback(self,data):
    #the actual string is called by data.data. update the appropriate class-owned variable.
    self.robotshot = data.data


  def loop(self,event):
    #this function runs over and over again at dt.
    #do stuff based on states. 

    #update the target controller
    self.targets.update(self.robotshot)
    self.conditionpub.publish(self.targets.trialType)
    self.statepub.publish(self.targets.state)

    if ((self.targets.trialType == 'EL') or (self.targets.trialType == 'CL')):
        if( (self.targets.state == 'target' )):
            self.ser.write("!"+str(self.outPosition)+","+str(self.inPosition)+"\r\n")
        else:
            self.ser.write("!"+str(self.inPosition)+","+str(self.inPosition)+"\r\n")
    elif ((self.targets.trialType == 'ER') or (self.targets.trialType == 'CR')):
        if( (self.targets.state == 'target' )):
            self.ser.write("!"+str(self.inPosition)+","+str(self.outPosition)+"\r\n")
        else:
            self.ser.write("!"+str(self.inPosition)+","+str(self.inPosition)+"\r\n")
    
    timenow = rospy.Time.now()

    # now set up output messages
    self.br.sendTransform((self.xoffset,self.yoffset,self.zoffset),tf.transformations.quaternion_from_euler(0,0,0),timenow,'/target_coord_system','/robot_static_cmd')


    targetmarker = Marker()
    targetmarker.header.frame_id='/target_coord_system'
    targetmarker.header.stamp = timenow
    targetmarker.type = targetmarker.SPHERE
    # targetmarker.mesh_resource = 'package://fishgantry_ros/meshes/static.dae'
    # targetmarker.mesh_use_embedded_materials = True
    targetmarker.action = targetmarker.MODIFY
    targetmarker.scale.x = .0254
    targetmarker.scale.y = .0254
    targetmarker.scale.z = .0254
    tempquat = tf.transformations.quaternion_from_euler(0,0,0)#this is RELATIVE TO FISH ORIENTATION IN TF (does the mesh have a rotation?)
    targetmarker.pose.orientation.w = tempquat[3]
    targetmarker.pose.orientation.x = tempquat[0]
    targetmarker.pose.orientation.y = tempquat[1]
    targetmarker.pose.orientation.z = tempquat[2]
    targetmarker.pose.position.x = 0
    targetmarker.pose.position.y = 0
    targetmarker.pose.position.z = 0
    self.targetMarkerpub.publish(targetmarker)

    #now send the pose of the target to the robot brain

    tpose = PoseStamped()
    tpose.header.stamp = timenow
    tpose.pose.position.x = self.targets.pose.x-self.xoffset
    tpose.pose.position.y = self.targets.pose.y-self.yoffset
    tpose.pose.position.z = self.targets.pose.z-self.zoffset
    self.targetposepub.publish(tpose)






    
      
#main function
def main(args):
  rospy.init_node('targets_node', anonymous=True)
  my_node = TargetsNode()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"

if __name__ == '__main__':
    main(sys.argv) 