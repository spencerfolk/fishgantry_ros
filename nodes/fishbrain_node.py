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
from fishgantry_ros.srv import setState
from numpy import *

from PTW import *
from PTWSocial import *

class FishBrain():
    def __init__(self):

        # state 1: manual control
        # state 2: PTW antisocial
        # state 3: PTW social
        self.initialized = False
        self.manpath = MarkovChain()
        self.manpath.updateGeometry(.40,.15,.10)

        self.manpath_social = MarkovChainSocial(theta_w=27.4,sigma_u=0.0000001,sigma_zdot=0.0000001,sigma_w=0.000001,sigma_o=0.00000001,dfish=0.1,dist_K=.001,yaw_K = .3)
        self.manpath_social.updateGeometry(.40,.15,.10)

        self.enabled = True
        self.home = False #make sure this gets implemented as a one-shot
        # self.state = 1
        #what is our friend fish's position
        self.otherfish = PoseStamped()
        self.listener = tf.TransformListener()



        self.timenow = rospy.Time.now()

        self.feedbacksub = rospy.Subscriber("/fishgantry/robotpose",PoseStamped,self.feedbackcallback)
        self.fishposesub = rospy.Publisher("/fishtracker/kalmanfishpose",PoseStamped,self.fishcallback,queue_size=1)

        self.restingposesub = rospy.Subscriber("/fishgantry/restingpose",PoseStamped,self.restingposecallback,queue_size=1)
        self.teleopposesub = rospy.Subscriber("/fishgantry/teleoppose",PoseStamped,self.teleopposecallback,queue_size=1)

        self.restingpose = PoseStamped();
        self.restingpose.pose.position.x=0
        self.restingpose.pose.position.y=0
        self.restingpose.pose.position.z=0
        self.restingpose.pose.orientation.x=0
        self.restingpose.pose.orientation.y=0
        self.restingpose.pose.orientation.z=0
        self.restingpose.pose.orientation.w=0

        self.feedbackPose = PoseStamped()
        self.teleoppose = PoseStamped()

        ## this is the frequency and amplitude for the "noise state" motion. Should be a small position that produces cyclical noise
        self.noisemotionfreq = 3 # rad/s, 
        self.noisemotionamp = 0.01#1cm command changes. Shouldn't result in ACTUAL perceptible motion


        #create a rate limit for moves to the resting position.
        self.linrate = 0.005 #meters per second
        self.yawratelimit = 0.5 #radians per second

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

        rospy.Service('set_robot_state',setState,self.set_robot_state)
        # rospy.Service('robot_enable',std_srvs.srv.Trigger,self.robot_enable)
        # rospy.Service('robot_disable',std_srvs.srv.Trigger,self.robot_disable)
        
        

        #state 1 is manual, state 2 is antisocial, state 3 is social
        self.state = 1
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


    def restingposecallback(self,data):
        self.restingpose = data

    def teleopposecallback(self,data):
        self.teleoppose = data

    def fishcallback(self,data):
        self.otherfish = data
        

    def rateLimit(self,x,y,z,pitch,yaw,tail):
        #this function takes a goal pose and a current robot pose. If the request requires too fast of a move, 
        #this function returns the best the robot can do under the current rate limits.
        quat = [self.feedbackPose.pose.orientation.x,self.feedbackPose.pose.orientation.y,self.feedbackPose.pose.orientation.z,self.feedbackPose.pose.orientation.w]

        froll,fpitch,fyaw = tf.transformations.euler_from_quaternion(quat)
        if((abs(x-self.feedbackPose.pose.position.x)/self.dt)>self.rateLimit):
            newx+=sign(x-self.feedbackPose.pose.position.x)*self.rateLimit*self.dt
            rospy.logwarn("RATE LIMIT X")
        else:
            newx = x
        if((abs(x-self.feedbackPose.pose.position.y)/self.dt)>self.rateLimit):
            newy+=sign(y-self.feedbackPose.pose.position.y)*self.rateLimit*self.dt
            rospy.logwarn("RATE LIMIT Y")
        else:
            newy = y
        if((abs(z-self.feedbackPose.pose.position.z)/self.dt)>self.rateLimit):
            newz+=sign(z-self.feedbackPose.pose.position.z)*self.rateLimit*self.dt
            rospy.logwarn("RATE LIMIT Z")
        else:
            newz = z
        if((abs(pitch-fpitch)/self.dt)>self.rateLimit):
            newpitch+=sign(pitch-fpitch)*self.rateLimit*self.dt
        else:
            newpitch = pitch
        if((abs(yaw-fyaw)/self.dt)>self.rateLimit):
            newyaw+=sign(yaw-fyaw)*self.rateLimit*self.dt
        else:
            newyaw = yaw

        return newx,newy,newz,newpitch,newyaw,tail        

    def feedbackcallback(self,data):
        if not self.initialized:
            # self.manpath.x = data.pose.position.x
            # self.manpath.y = data.pose.position.y
            # self.manpath.z = data.pose.position.z
            # r,p,a = tf.transformations.euler_from_quaternion([data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w])
            # self.manpath.pitchnow = p
            # self.manpath.psi = a
            self.initialized = True;
        self.feedbackPose = data

    def loop(self,data):
        #conditions for robot experiments
        # 1: No robot present... state 1 here is teleop.
        # 2: Robot Present, doing nothing
        # 3: Robot Present, moving only enough to make motor noise
        # 4: Robot Present, moving only enough to make motor noise and moving tail
        # 5: Robot follows pre-recorded path
        # 6: Robot uses PTW antisocial
        # 7: Robot uses PTW social
        if self.initialized:
            if self.state == 6:
                if self.enabled:
                    #self.pose.pose.position.x,self.pose.pose.position.y,self.pose.pose.position.z,pitch,yaw,tail = self.manpath.drivePersistentFish(self.dt)
                    x,y,z,pitch,yaw,tail = self.manpath.drivePersistentFish(self.dt)
                    self.pose.pose.position.x,self.pose.pose.position.y,self.pose.pose.position.z,pitch,yaw,tail = self.rateLimit(x,y,z,pitch,yaw,tail)

                    quat = tf.transformations.quaternion_from_euler(0,pitch,yaw)
                    self.pose.pose.orientation.x,self.pose.pose.orientation.y,self.pose.pose.orientation.z,self.pose.pose.orientation.w = quat
                    self.pose.header.stamp = rospy.Time.now()
                    self.goalpose_pub.publish(self.pose)
                    #rospy.logwarn("trying to publish pose from antisocial")
            elif self.state ==7:
                try:
                    fishtrans,fishrot = self.listener.lookupTransform('/fishkalman','/robot_static_cmd',rospy.Time(0))
                    fishroll,fishpitch,fishyaw = tf.transformations.euler_from_quaternion(fishrot)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    fishtrans = [0,0,0]
                    fishyaw = 0
                    # rospy.logwarn('no transform yet to fishkalman')
                if self.enabled:
                    friendpos = [fishtrans[0],fishtrans[1],fishyaw]
                    x,y,z,pitch,yaw,tail = self.manpath_social.drivePersistentFish(self.dt,friendpos=friendpos)
                    self.pose.pose.position.x,self.pose.pose.position.y,self.pose.pose.position.z,pitch,yaw,tail = self.rateLimit(x,y,z,pitch,yaw,tail)
                    # rospy.logwarn("friend pos is "+str(friendpos))
                    quat = tf.transformations.quaternion_from_euler(0,pitch,yaw)
                    self.pose.pose.orientation.x,self.pose.pose.orientation.y,self.pose.pose.orientation.z,self.pose.pose.orientation.w = quat
                    self.pose.header.stamp = rospy.Time.now()
                    self.goalpose_pub.publish(self.pose)
                    #rospy.logwarn("trying to publish pose from antisocial")
            elif self.state==1:
                #state 1 is the dummy state. use for teleop
                if self.enabled:
                    x,y,z = self.teleoppose.pose.position.x,self.teleoppose.pose.position.y,self.teleoppose.pose.position.z
                    quat = [self.restingpose.pose.orientation.x,self.restingpose.pose.orientation.y,self.restingpose.pose.orientation.z,self.restingpose.pose.orientation.w]
                    roll,pitch,yaw = tf.transformations.euler_from_quaternion(quat)
                    tail = 0
                    x,y,z,pitch,yaw,tail = self.rateLimit(x,y,z,pitch,yaw,tail)
                    self.pose.pose.position.x = x
                    self.pose.pose.position.y = y
                    self.pose.pose.position.z = z
                    quat = tf.transformations.quaternion_from_euler(0,pitch,yaw)
                    self.pose.pose.orientation.x = quat[0]
                    self.pose.pose.orientation.y = quat[1]
                    self.pose.pose.orientation.z = quat[2]
                    self.pose.pose.orientation.w = quat[3]
                    self.pose.header.stamp = rospy.Time.now()
                    #rospy.logwarn(self.pose)
                    self.goalpose_pub.publish(self.pose)
            elif self.state==2:
                #state 2 is the doing nothing state. This position gets set via the launch file.
                if self.enabled:
                    x,y,z = self.teleoppose.pose.position.x,self.teleoppose.pose.position.y,self.teleoppose.pose.position.z
                    quat = [self.restingpose.pose.orientation.x,self.restingpose.pose.orientation.y,self.restingpose.pose.orientation.z,self.restingpose.pose.orientation.w]
                    roll,pitch,yaw = tf.transformations.euler_from_quaternion(quat)
                    tail = 0
                    x,y,z,pitch,yaw,tail = self.rateLimit(x,y,z,pitch,yaw,tail)
                    self.pose.pose.position.x = x
                    self.pose.pose.position.y = y
                    self.pose.pose.position.z = z
                    quat = tf.transformations.quaternion_from_euler(0,pitch,yaw)
                    self.pose.pose.orientation.x = quat[0]
                    self.pose.pose.orientation.y = quat[1]
                    self.pose.pose.orientation.z = quat[2]
                    self.pose.pose.orientation.w = quat[3]
                    self.pose.header.stamp = rospy.Time.now()
                    self.goalpose_pub.publish(self.pose)

            elif self.state==3:
                #state 3 is the state where the robot moves cyclically in a tiny motion to just make noise
                if self.enabled:
                    x = self.restingpose.pose.position.x + self.noisemotionamp*sin(self.noisemotionfreq*time.time())
                    y = self.restingpose.pose.position.x + self.noisemotionamp*sin(self.noisemotionfreq*time.time())
                    z = self.restingpose.pose.position.z
                    quat = [self.restingpose.pose.orientation.x,self.restingpose.pose.orientation.y,self.restingpose.pose.orientation.z,self.restingpose.pose.orientation.w]
                    roll,pitch,yaw = tf.transformations.euler_from_quaternion(quat)
                    tail = 0
                    x,y,z,pitch,yaw,tail = self.rateLimit(x,y,z,pitch,yaw,tail)
                    self.pose.pose.position.x = x
                    self.pose.pose.position.y = y
                    self.pose.pose.position.z = z
                    quat = tf.transformations.quaternion_from_euler(0,pitch,yaw)
                    self.pose.pose.orientation.x = quat[0]
                    self.pose.pose.orientation.y = quat[1]
                    self.pose.pose.orientation.z = quat[2]
                    self.pose.pose.orientation.w = quat[3]
                    self.pose.header.stamp = rospy.Time.now()
                    self.goalpose_pub.publish(self.pose)
            else:
                rospy.logwarn("INVALID STATE REQUEST RECEIVED FOR ROBOT")



            
        else:
            self.goalpose_pub.publish(self.pose)
            rospy.logwarn("not initialized yet. publishing to get response")


    def slowloop(self,data):
        if self.state==2:
            self.manpath.updateCurrentState()
        elif self.state==3:
            self.manpath_social.updateCurrentState()

    def set_robot_state(self,data):
        self.state=data.data
    # def set_manual(self,data):
    #     self.state = 1

    # def set_antisocial(self,data):
    #     self.state = 2

    # def set_social(self,data):
    #     self.state = 3


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