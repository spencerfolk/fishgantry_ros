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
from fishgantry_ros.srv import setState,setStateResponse
from std_msgs.msg import *
from numpy import *
import time

from PTW import *
from PTWSocial import *
from hunting_fishbrain.hunting_fishbrain.HybridFishBrain import *
import copy

class FishBrainManager():
    def __init__(self):

        # state 1: manual control
        # state 2: PTW antisocial
        # state 3: PTW social

        ### set up hunter for summer 2020 experiments

        tc = TargetingController()
        sc = PTWSwimController(muu=0.02,muw=0.2,muz = 0.0, nu=.01,nw=.5, nz = 0.05,tauu=0.1,tauw = .1,tauz = .1)
        cc = PTWSwimController(muu=0.0,muw=0.0,muz = 0.0, nu=0,nw=0, nz = 0,tauu=.1,tauw = .1,tauz = .1)

        goalTarg = FishState(.85,.15,.15,0,0) #the target has no inherent pitch or yaw requirement

        TankBounds =[0,.38,0,.14,-.1,0]

        self.huntbrain = FishBrain(TranMat=[[.95,.05],[.1,.9]])
        self.huntcont = FishControlManager(goalTarg,sc,cc,tc,TankBounds)
        self.fbpose = FishState()
        self.oldfbpose = FishState()

        ### old stuff
        self.initialized = False
        self.manpath = MarkovChain()
        self.manpath.updateGeometry(.38,.14,.10)

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

        self.targetposepub = rospy.Subscriber("/fishtarget_ros/targetpose",PoseStamped,self.targetposecallback,queue_size = 1)
        self.targetstatepub = rospy.Subscriber("/fishtarget_ros/target_is_out",String,self.targetstatecallback,queue_size=1)
        self.experimentalconditionsub = rospy.Subscriber("/fishtarget_ros/experimental_condition",String,self.experimentalconditioncallback,queue_size=1)

        self.expcond = "CL"
        self.targetstate = "waiting"
        self.targetpose = Pose()

        self.restingpose = PoseStamped();
        self.restingpose.pose.position.x=0.2
        self.restingpose.pose.position.y=.07
        self.restingpose.pose.position.z=0
        self.restingpose.pose.orientation.x=0
        self.restingpose.pose.orientation.y=0
        self.restingpose.pose.orientation.z=0
        self.restingpose.pose.orientation.w=0

        self.feedbackPose = PoseStamped()
        self.feedbackPose.pose.position.x=0
        self.feedbackPose.pose.position.y=0
        self.feedbackPose.pose.position.z=0
        self.feedbackPose.pose.orientation.x=0
        self.feedbackPose.pose.orientation.y=0
        self.feedbackPose.pose.orientation.z=0
        self.feedbackPose.pose.orientation.w=0

        self.dtfeedback = .1
        self.feedbacktime = 0
        self.oldfeedbacktime = -.1

        self.teleoppose = PoseStamped()
        self.teleoppose.pose.position.x=0
        self.teleoppose.pose.position.y=0
        self.teleoppose.pose.position.z=0
        self.teleoppose.pose.orientation.x=0
        self.teleoppose.pose.orientation.y=0
        self.teleoppose.pose.orientation.z=0
        self.teleoppose.pose.orientation.w=0

        ## this is the frequency and amplitude for the "noise state" motion. Should be a small position that produces cyclical noise
        self.noisemotionfreq = 3 # rad/s, 
        self.noisemotionamp = 0.0005#1cm command changes. Shouldn't result in ACTUAL perceptible motion

        self.tailfreq = 6
        self.tailamp = 20


        #create a rate limit for moves to the resting position.
        self.linrate = 0.05 #meters per second
        self.yawratelimit = 0.5 #radians per second

        self.goalpose_pub = rospy.Publisher("/fishgantry/commandpose",PoseStamped,queue_size=1)
        self.tailpose_pub = rospy.Publisher("/fishgantry/tailpose",PoseStamped,queue_size=1)
        self.laps_pub = rospy.Publisher("/fishgantry/laps",Int32,queue_size=1)
        self.rawyaw_pub = rospy.Publisher("/fishgantry/rawyaw",Float32,queue_size=1)
        self.squirtpose_pub = rospy.Publisher("fishgantry_ros/squirtpose",PoseStamped,queue_size=1)

        self.robotshotpub = rospy.Publisher("/fishgantry/robotshot",Bool,queue_size = 1)
        self.robotshot = False

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
        self.filepath = self.package_path+"/playback/robotmotion.txt"
        self.fileplayer = RecordedPath(self.filepath,self.dt)

        self.huntbrainstate = "none"
        self.oldhuntbrainstate = "none"

    def targetstatecallback(self,data):
        self.targetstate = data.data

    def targetposecallback(self,data):
        self.targetpose = data.pose

    def experimentalconditioncallback(self,data):
        self.expcond = data.data

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
        if((abs(x-self.pose.pose.position.x)/self.dt)>self.linrate):
            newx=self.pose.pose.position.x+sign(x-self.pose.pose.position.x)*self.linrate*self.dt
            # rospy.logwarn(self.pose.pose.position.x+ sign(x-self.pose.pose.position.x)*self.linrate*self.dt)
        else:
            # rospy.logwarn((abs(x-self.pose.pose.position.x)/self.dt)>self.linrate)
            newx = x
        if((abs(x-self.pose.pose.position.y)/self.dt)>self.linrate):
            newy=self.pose.pose.position.y +sign(y-self.pose.pose.position.y)*self.linrate*self.dt
            # rospy.logwarn(newy)
        else:
            newy = y
        if((abs(z-self.pose.pose.position.z)/self.dt)>self.linrate):
            newz= self.pose.pose.position.z +sign(z-self.pose.pose.position.z)*self.linrate*self.dt
            # rospy.logwarn(newz)
        else:
            newz = z
        if((abs(pitch-fpitch)/self.dt)>self.yawratelimit):
            newpitch=pitch+sign(pitch-fpitch)*self.yawratelimit*self.dt
        else:
            newpitch = pitch
        if((abs(yaw-fyaw)/self.dt)>self.yawratelimit):
            newyaw=yaw+sign(yaw-fyaw)*self.yawratelimit*self.dt
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

        self.feedbacktime = time.time()

        self.dtfeedback = self.feedbacktime-self.oldfeedbacktime
        self.oldfeedbacktime = self.feedbacktime
        self.feedbackPose = data

        fbr,fbp,fby = tf.transformations.euler_from_quaternion([self.feedbackPose.pose.orientation.x,self.feedbackPose.pose.orientation.y,self.feedbackPose.pose.orientation.z,self.feedbackPose.pose.orientation.w])
        self.fbpose.x,self.fbpose.y,self.fbpose.z,self.fbpose.tilt,self.fbpose.psi = self.feedbackPose.pose.position.x,self.feedbackPose.pose.position.y,self.feedbackPose.pose.position.z,fbp,fby
        self.fbpose.xdot = (self.fbpose.x-self.oldfbpose.x)/self.dtfeedback
        self.fbpose.ydot = (self.fbpose.y-self.oldfbpose.y)/self.dtfeedback
        self.fbpose.U = (self.fbpose.xdot**2+self.fbpose.ydot**2)**.5
        self.fbpose.zdot = (self.fbpose.z-self.oldfbpose.z)/self.dtfeedback
        self.fbpose.Tiltdot = (self.fbpose.tilt-self.oldfbpose.tilt)/self.dtfeedback
        self.fbpose.Psidot = (self.fbpose.psi-self.oldfbpose.psi)/self.dtfeedback
        self.oldfbpose = copy.deepcopy(self.fbpose)
        # rospy.logwarn([self.fbpose.xdot,self.fbpose.ydot,self.fbpose.zdot, self.fbpose.Psidot, self.fbpose.U])
        # rospy.logwarn([data.pose.position.x,data.pose.position.y,data.pose.position.z])
        # rospy.logwarn([fbr,fbp,fby])

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
            if self.state == 9:

                #decide whether robot should be hunting
                if((self.expcond[0]=="E") and (self.targetstate == "target")):
                    hunt = True
                else:
                    hunt = False


                # update fish controller manager to know where the target is 
                self.huntcont.goal = FishState(self.targetpose.position.x,self.targetpose.position.y,self.targetpose.position.z,0,0)
                
                if self.enabled:

                    command,u,e = self.huntcont.getGantryCommand(self.huntbrain.state,self.fbpose,time.time())

                    fishstate,fishshot = self.huntbrain.update(hunt,e,time.time())
                    #rospy.logwarn(self.huntbrain.state)
                    x,y,z,pitch,yaw = command.x,command.y,command.z,command.tilt,command.psi #tail is not yet implemented

                    rospy.logwarn([command.y])
                    tail = 0
                    self.pose.pose.position.x,self.pose.pose.position.y,self.pose.pose.position.z,pitch,yaw,tail = self.rateLimit(x,y,z,pitch,yaw,tail)
                    quat = tf.transformations.quaternion_from_euler(0,pitch,yaw)
                    self.pose.pose.orientation.x,self.pose.pose.orientation.y,self.pose.pose.orientation.z,self.pose.pose.orientation.w = quat
                    self.pose.header.stamp = rospy.Time.now()
                    self.goalpose_pub.publish(self.pose)

                    tailcommand = tail
                    tailposemsg = PoseStamped()
                    tailposemsg.header.stamp = rospy.Time.now()
                    tailposemsg.pose.orientation.z = tailcommand
                    self.tailpose_pub.publish(tailposemsg)


                    self.huntbrainstate = self.huntbrain.state
                    if((self.huntbrainstate=="huntcapture") and (self.oldhuntbrainstate == "hunttilt")):
                        squirtcommand = 180
                    else:
                        squirtcommand = 0

                    squirtposemsg = PoseStamped()
                    squirtposemsg.header.stamp = rospy.Time.now()
                    squirtposemsg.pose.orientation.z = squirtcommand
                    self.squirtpose_pub.publish(squirtposemsg)

            elif self.state == 8:
                if self.enabled:
                    #this is the binary choice experiment state. We need to flip flop between a PTW and a guided PTW.
                    #The guided PTW must end up at the center of the tank facing EITHER left or right, and then
                    #The robot will execute a tilt.

                    #self.pose.pose.position.x,self.pose.pose.position.y,self.pose.pose.position.z,pitch,yaw,tail = self.manpath.drivePersistentFish(self.dt)
                    x,y,z,pitch,yaw,tail = self.manpath.drivePersistentFish(self.dt)
                    self.pose.pose.position.x,self.pose.pose.position.y,self.pose.pose.position.z,pitch,yaw,tail = self.rateLimit(x,y,z,pitch,yaw,tail)

                    quat = tf.transformations.quaternion_from_euler(0,pitch,yaw)
                    self.pose.pose.orientation.x,self.pose.pose.orientation.y,self.pose.pose.orientation.z,self.pose.pose.orientation.w = quat
                    self.pose.header.stamp = rospy.Time.now()
                    self.goalpose_pub.publish(self.pose)
                    #rospy.logwarn("trying to publish pose from antisocial")
            elif self.state == 6:
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
                    quat = [self.teleoppose.pose.orientation.x,self.teleoppose.pose.orientation.y,self.teleoppose.pose.orientation.z,self.teleoppose.pose.orientation.w]
                    roll,pitch,yaw = tf.transformations.euler_from_quaternion(quat)
                    tail = 0
                    nx,ny,nz,npitch,nyaw,ntail = self.rateLimit(x,y,z,pitch,yaw,tail)
                    #rospy.logwarn(self.feedbackPose.pose.position.x)
                    self.pose.pose.position.x = nx
                    self.pose.pose.position.y = ny
                    self.pose.pose.position.z = nz
                    quat = tf.transformations.quaternion_from_euler(0,npitch,nyaw)
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
                    x,y,z = self.restingpose.pose.position.x,self.restingpose.pose.position.y,self.restingpose.pose.position.z
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
                    y = self.restingpose.pose.position.y + self.noisemotionamp*sin(self.noisemotionfreq*time.time())
                    z = self.restingpose.pose.position.z
                    tailcommand = self.tailamp*sin(self.tailfreq*time.time())
                    tailposemsg = PoseStamped()
                    tailposemsg.header.stamp = rospy.Time.now()
                    tailposemsg.pose.orientation.z = tailcommand

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
                    self.tailpose_pub.publish(tailposemsg)

            elif self.state==5:
                #state 5 is the state where the robot moves in a path defined by a recording of fish position (in a file)
                if self.enabled:
                    x,y,z,pitch,yaw,tail = self.fileplayer.update(self.dt)
                    
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
                    # tailcommand = self.tailamp*sin(self.tailfreq*time.time())
                    tailposemsg = PoseStamped()
                    tailposemsg.header.stamp = rospy.Time.now()
                    tailposemsg.pose.orientation.z = tail
                    lapsmsg = Int32()
                    lapsmsg.data = self.fileplayer.laps
                    self.laps_pub.publish(lapsmsg)
                    self.tailpose_pub.publish(tailposemsg)
                    self.goalpose_pub.publish(self.pose)
            else:
                rospy.logwarn("INVALID STATE REQUEST RECEIVED FOR ROBOT")
                #rospy.logwarn(self.state)
                #pass
                #rospy.logwarn(self.state)



            
        else:
            self.goalpose_pub.publish(self.pose)
            rospy.logwarn("not initialized yet. publishing to get response")


    def slowloop(self,data):
        if self.state==2:
            self.manpath.updateCurrentState()
        elif self.state==3:
            self.manpath_social.updateCurrentState()

    def set_robot_state(self,data):
        self.state=data.state
        if(self.state==1):
            return setStateResponse("Teleop Mode")
        elif(self.state==2):
            return setStateResponse("Do nothing in tank center Mode")
        elif(self.state==3):
            return setStateResponse("Small motion for motor noise Mode")
        elif(self.state==4):
            return setStateResponse("NOT IMPLEMENTED YET: Motor noise and tail")
        elif(self.state==5):
            return setStateResponse("file playback mode: file in playback/robotmotion.txt")
        elif(self.state==6):
            return setStateResponse("Antisocial Persistent Turning Walker Mode")
        elif(self.state==7):
            return setStateResponse("Social Persistent Turning Walker Mode")
        elif(self.state==8):
            return setStateResponse("Binary Shooting Direction Mode")
        elif(self.state==9):
            return setStateResponse("Hunting Experiment Mode")
        else:
            return setStateResponse("INVALID: TRY A NUMBER 1-9")
        
        # rospy.logwarn(self.state)
        # return setStateResponse(1)
    # def set_manual(self,data):
    #     self.state = 1

    # def set_antisocial(self,data):
    #     self.state = 2

    # def set_social(self,data):
    #     self.state = 3


    #main function

class RecordedPath():
    def __init__(self,fname,dt):

        if fname is not None:
            self.tailtheta = 0.
            self.tailangle = 0.
            

            self.xnow = 0.
            self.ynow = 0.
            self.znow = 0.
            self.pitchnow =0.
            self.yawnow = 0.
            #data should come in as t,x,y,z
            self.data = loadtxt(fname)
            self.tdata = self.data[:,0]
            self.xdata = self.data[:,1]
            self.ydata = self.data[:,2]
            self.zdata = self.data[:,3]
            self.pitchdata = self.data[:,4]
            self.yawdata = self.data[:,5]
            self.taildata = self.data[:,6]
            self.lapsdata = fix(self.yawdata/(2*pi))
            self.laps = 0
            # self.yawdata = medfilt(self.yawdata,5)
            #zero out the time just in case it does not start at zero
            self.tdata = self.tdata-self.tdata[0]
            self.tnow = 0.

        else:
            print "No valid file!"

    def update(self,dt):
        self.tnow +=dt
        if(self.tnow<=self.tdata[-1]):
            self.xnow = interp(self.tnow,self.tdata,self.xdata)
            self.ynow = interp(self.tnow,self.tdata,self.ydata)
            self.znow = interp(self.tnow,self.tdata,self.zdata)
            # self.pitchnow = interp(self.tnow,self.tdata,self.pitchdata)
            self.yawnow = interp(self.tnow,self.tdata,self.yawdata)
            self.pitchnow = interp(self.tnow,self.tdata,self.pitchdata)
            self.tailangle = interp(self.tnow,self.tdata,self.taildata)
            self.laps = fix(interp(self.tnow,self.tdata,self.lapsdata))
            print self.laps
        
        return self.xnow,self.ynow,self.znow,self.pitchnow,self.yawnow,self.tailangle

def main(args):
  rospy.init_node('fishbrain_node', anonymous=True)
  fishbrain = FishBrainManager()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"

if __name__ == '__main__':
    main(sys.argv) 