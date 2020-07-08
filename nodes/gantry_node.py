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
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Int32
import std_srvs.srv
from numpy import *

class FishGantry():
  def __init__(self):
    self.laps = 0
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
    self.squirtpose_sub = rospy.Subscriber("/fishgantry/squirtpose",PoseStamped,self.squirtCallback)
    self.laps_sub = rospy.Subscriber("/fishgantry/laps",Int32,self.lapscallback)
    #initialize a command position
    self.command = PoseStamped()
    self.command.header.stamp = rospy.Time.now()
    self.command.pose.position.x = 0
    self.command.pose.position.y = 0
    self.command.pose.position.z = 0
    self.command.pose.orientation.z = 0
    self.command.pose.orientation.x = 0
    self.tailcommand = 0;
    self.squirtcommand =145;
    self.rollcommand,self.pitchcommand,self.yawcommand=0,0,0

    self.br = tf.TransformBroadcaster()

    rospy.Service('enable_motors',std_srvs.srv.Trigger,self.enable_motors)
    rospy.Service('disable_motors',std_srvs.srv.Trigger,self.disable_motors)
    rospy.Service('home_axes',std_srvs.srv.Trigger,self.home)


    #main loop runs on a timer, which ensures that we get timely updates from the gantry arduino
    rospy.Timer(rospy.Duration(.01),self.loop,oneshot=False) #timer callback (math) allows filter to run at constant time
    rospy.Timer(rospy.Duration(.1),self.slowloop,oneshot=False)
    #subscribers (inputs)
    #construct the file name for our text output file
    rospack = rospkg.RosPack()
    # get the file path for rospy_tutorials
    self.package_path=rospack.get_path('fishgantry_ros')

  def lapscallback(self,data):
    # self.laps = float(data.data)
    rospy.logwarn(self.laps)
  def tailCallback(self,data):
    self.tailcommand = data.pose.orientation.z

  def squirtCallback(self,data):
    self.squirtcommand = data.pose.orientation.z




  def commandCallback(self,data):
    self.command.header.stamp = data.header.stamp
    self.command.pose.position.x = data.pose.position.x
    self.command.pose.position.y = data.pose.position.y
    self.command.pose.position.z = data.pose.position.z
    self.command.pose.orientation.z = data.pose.orientation.z
    self.command.pose.orientation.x = data.pose.orientation.x
    self.command.pose.orientation.y = data.pose.orientation.y
    self.command.pose.orientation.w = data.pose.orientation.w

    self.rollcommand,pitchcommand,yawcommand = tf.transformations.euler_from_quaternion([self.command.pose.orientation.x,self.command.pose.orientation.y,self.command.pose.orientation.z,self.command.pose.orientation.w])
    self.pitchcommand = -pitchcommand
    if (yawcommand-self.yawcommand)>pi:
        self.laps-=1
    if (yawcommand-self.yawcommand)<-pi:
        self.laps+=1
    self.yawcommand = yawcommand
    # rospy.logwarn(str(self.yawcommand))
    #self.command.pose.orientation.roll,self.command.pose.orientation.pitch,self.command.pose.orientation.yaw#
    #print "received: "+str(self.command.pose.position.x)

  def enable_motors(self,event):
    encommand = -333.30
    serstring = '!'+"{0:.3f}".format(encommand)+','+"{0:.3f}".format(encommand)+','+"{0:.3f}".format(encommand)+','+"{0:.3f}".format(encommand)+','+"{0:.3f}".format(encommand)+','+"{0:.3f}".format(0)+','+"{0:.3f}".format(0)+'\r\n'
    print "sending: "+serstring
    self.ser.write(serstring)
    line = self.ser.readline()
    print line

  def disable_motors(self,event):
    discommand = -333.30
    serstring = '!'+"{0:.3f}".format(discommand)+','+"{0:.3f}".format(discommand)+','+"{0:.3f}".format(discommand)+','+"{0:.3f}".format(discommand)+','+"{0:.3f}".format(discommand)+','+"{0:.3f}".format(0)+','+"{0:.3f}".format(0)+'\r\n'
    print "sending: "+serstring
    self.ser.write(serstring)
    line = self.ser.readline()
    print line
  def home(self,event):
    homecommand = -333.30
    serstring = '!'+"{0:.3f}".format(homecommand)+','+"{0:.3f}".format(homecommand)+','+"{0:.3f}".format(homecommand)+','+"{0:.3f}".format(homecommand)+','+"{0:.3f}".format(homecommand)+','+"{0:.3f}".format(0)+','+"{0:.3f}".format(0)+'\r\n'
    print "sending: "+serstring
    self.ser.write(serstring)
    line = self.ser.readline()
    print line

  def loop(self,event):
    rospy.logwarn(self.squirtcommand)
    serstring = '!'+"{0:.3f}".format(self.command.pose.position.x)+','+"{0:.3f}".format(self.command.pose.position.y)+','+"{0:.3f}".format(self.command.pose.position.z)+','+"{0:.3f}".format(self.pitchcommand)+','+"{0:.3f}".format(self.yawcommand+self.laps*2*pi)+','+"{0:.3f}".format(self.tailcommand)+','+"{0:.3f}".format(self.squirtcommand)+'\r\n'
    # print "sending: "+serstring
    self.ser.write(serstring)
    line = self.ser.readline()
    # print line
    linestrip = line.strip('\r\n')
    linesplit = line.split()
    if len(linesplit)>=3:
      #print shotslast, arduinonumshots,shotslast==(arduinonumshots+1)
      
      try:
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = float(linesplit[1])
        msg.pose.position.y = float(linesplit[2])
        msg.pose.position.z = float(linesplit[3])
        pitchback = float(linesplit[3])
        yawback = float(linesplit[4])

        quat = tf.transformations.quaternion_from_euler(0,pitchback,yawback)
        msg.pose.orientation.x=quat[0]
        msg.pose.orientation.y=quat[1]
        msg.pose.orientation.z=quat[2]
        msg.pose.orientation.w=quat[3]

        #msg.arduino_time = float(linesplit[2])
        #msg.tailfreq= float(linesplit[3])
        #msg.tailamp = float(linesplit[4])
        #msg.tailangle = float(linesplit[5])
        #print "publishing"
        self.pub.publish(msg)
              
      except:
        print "OOPS! BAD LINE"
        #ef.write("problem  with serial line"+"\r\n")
  def slowloop(self,event):
    
    timenow = rospy.Time.now()

    framemarker = Marker()
    framemarker.header.frame_id='/robot_static_cmd'
    framemarker.header.stamp = timenow
    framemarker.type = framemarker.MESH_RESOURCE
    framemarker.mesh_resource = 'package://fishgantry_ros/meshes/static.dae'
    framemarker.mesh_use_embedded_materials = True
    framemarker.action = framemarker.MODIFY
    framemarker.scale.x = .0254
    framemarker.scale.y = .0254
    framemarker.scale.z = .0254
    tempquat = tf.transformations.quaternion_from_euler(0,0,0)#this is RELATIVE TO FISH ORIENTATION IN TF (does the mesh have a rotation?)
    framemarker.pose.orientation.w = tempquat[3]
    framemarker.pose.orientation.x = tempquat[0]
    framemarker.pose.orientation.y = tempquat[1]
    framemarker.pose.orientation.z = tempquat[2]
    framemarker.pose.position.x = 0
    framemarker.pose.position.y = 0
    framemarker.pose.position.z = 0
    # framemarker.color.r = .5
    # framemarker.color.g = .8
    # framemarker.color.b = .5
    framemarker.color.a = 0.0#transparency

    ymarker = Marker()
    ymarker.header.frame_id='/robot_y_cmd'
    ymarker.header.stamp = timenow
    ymarker.type = ymarker.MESH_RESOURCE
    ymarker.mesh_resource = 'package://fishgantry_ros/meshes/ymotiongroup.dae'
    ymarker.mesh_use_embedded_materials = True
    ymarker.action = ymarker.MODIFY
    ymarker.scale.x = .0254
    ymarker.scale.y = .0254
    ymarker.scale.z = .0254
    tempquat = tf.transformations.quaternion_from_euler(0,0,0)#this is RELATIVE TO FISH ORIENTATION IN TF (does the mesh have a rotation?)
    ymarker.pose.orientation.w = tempquat[3]
    ymarker.pose.orientation.x = tempquat[0]
    ymarker.pose.orientation.y = tempquat[1]
    ymarker.pose.orientation.z = tempquat[2]
    ymarker.pose.position.x = 0
    ymarker.pose.position.y = 0
    ymarker.pose.position.z = 0
    # ymarker.color.r = .5
    # ymarker.color.g = .8
    # ymarker.color.b = .5
    ymarker.color.a = 0.0#transparency

    xmarker = Marker()
    xmarker.header.frame_id='/robot_x_cmd'
    xmarker.header.stamp = timenow
    xmarker.type = xmarker.MESH_RESOURCE
    xmarker.mesh_resource = 'package://fishgantry_ros/meshes/xmotiongroup.dae'
    xmarker.mesh_use_embedded_materials = True
    xmarker.action = xmarker.MODIFY
    xmarker.scale.x = .0254
    xmarker.scale.y = .0254
    xmarker.scale.z = .0254
    tempquat = tf.transformations.quaternion_from_euler(0,0,0)#this is RELATIVE TO FISH ORIENTATION IN TF (does the mesh have a rotation?)
    xmarker.pose.orientation.w = tempquat[3]
    xmarker.pose.orientation.x = tempquat[0]
    xmarker.pose.orientation.y = tempquat[1]
    xmarker.pose.orientation.z = tempquat[2]
    xmarker.pose.position.x = 0
    xmarker.pose.position.y = 0
    xmarker.pose.position.z = 0
    # xmarker.color.r = .5
    # xmarker.color.g = .8
    # xmarker.color.b = .5
    xmarker.color.a = 0.0#transparency

    yawmarker = Marker()
    yawmarker.header.frame_id='/robot_yaw_cmd'
    yawmarker.header.stamp = timenow
    yawmarker.type = yawmarker.MESH_RESOURCE
    yawmarker.mesh_resource = 'package://fishgantry_ros/meshes/yawmotiongroup.dae'
    yawmarker.mesh_use_embedded_materials = True
    yawmarker.action = yawmarker.MODIFY
    yawmarker.scale.x = .0254
    yawmarker.scale.y = .0254
    yawmarker.scale.z = .0254
    tempquat = tf.transformations.quaternion_from_euler(0,0,0)#this is RELATIVE TO FISH ORIENTATION IN TF (does the mesh have a rotation?)
    yawmarker.pose.orientation.w = tempquat[3]
    yawmarker.pose.orientation.x = tempquat[0]
    yawmarker.pose.orientation.y = tempquat[1]
    yawmarker.pose.orientation.z = tempquat[2]
    yawmarker.pose.position.x = 0
    yawmarker.pose.position.y = 0
    yawmarker.pose.position.z = 0
    # yawmarker.color.r = .5
    # yawmarker.color.g = .8
    # yawmarker.color.b = .5
    yawmarker.color.a = 0.0#transparency

    zmarker = Marker()
    zmarker.header.frame_id='/robot_z_cmd'
    zmarker.header.stamp = timenow
    zmarker.type = zmarker.MESH_RESOURCE
    zmarker.mesh_resource = 'package://fishgantry_ros/meshes/zmotiongroup.dae'
    zmarker.mesh_use_embedded_materials = True
    zmarker.action = zmarker.MODIFY
    zmarker.scale.x = .0254
    zmarker.scale.y = .0254
    zmarker.scale.z = .0254
    tempquat = tf.transformations.quaternion_from_euler(0,0,0)#this is RELATIVE TO FISH ORIENTATION IN TF (does the mesh have a rotation?)
    zmarker.pose.orientation.w = tempquat[3]
    zmarker.pose.orientation.x = tempquat[0]
    zmarker.pose.orientation.y = tempquat[1]
    zmarker.pose.orientation.z = tempquat[2]
    zmarker.pose.position.x = 0
    zmarker.pose.position.y = 0
    zmarker.pose.position.z = -.929*.0254
    # zmarker.color.r = .5
    # zmarker.color.g = .8
    # zmarker.color.b = .5
    # zmarker.color.a = 1.0#transparency

    tiltmarker = Marker()
    tiltmarker.header.frame_id='/robot_pitch_cmd'
    tiltmarker.header.stamp = timenow
    tiltmarker.type = tiltmarker.MESH_RESOURCE
    tiltmarker.mesh_resource = 'package://fishgantry_ros/meshes/tiltmotiongroup.dae'
    tiltmarker.mesh_use_embedded_materials = True
    tiltmarker.action = tiltmarker.MODIFY
    tiltmarker.scale.x = .0254
    tiltmarker.scale.y = .0254
    tiltmarker.scale.z = .0254
    tempquat = tf.transformations.quaternion_from_euler(0,0,0)#this is RELATIVE TO FISH ORIENTATION IN TF (does the mesh have a rotation?)
    tiltmarker.pose.orientation.w = tempquat[3]
    tiltmarker.pose.orientation.x = tempquat[0]
    tiltmarker.pose.orientation.y = tempquat[1]
    tiltmarker.pose.orientation.z = tempquat[2]
    tiltmarker.pose.position.x = 0
    tiltmarker.pose.position.y = 0
    tiltmarker.pose.position.z = -.929*.0254
    # tiltmarker.color.r = .5
    # tiltmarker.color.g = .8
    # tiltmarker.color.b = .5
    tiltmarker.color.a = 0.0#transparency

    self.staticpubcmd.publish(framemarker)
    self.ypubcmd.publish(ymarker)
    self.xpubcmd.publish(xmarker)
    self.yawpubcmd.publish(yawmarker)
    self.zpubcmd.publish(zmarker)
    self.pitchpubcmd.publish(tiltmarker)

    ############################## now publish markers and transforms ############################3
    #publish transform from world to static frame CS.
    # self.br.sendTransform((-24.0*.0254,-9*.0254,(-18-6)*.0254),tf.transformations.quaternion_from_euler(0,0,-pi/2),rospy.Time.now(),'/robot_static_cmd','/world')
    # self.br.sendTransform((-.34,-.36,-.6),tf.transformations.quaternion_from_euler(0,0,-pi/2),rospy.Time.now(),'/robot_static_cmd','/world')
    self.br.sendTransform((-.44,-.36,-.6),tf.transformations.quaternion_from_euler(0,0,-pi/2),rospy.Time.now(),'/robot_static_cmd','/world')

    #publish transform from static to y motion
    self.br.sendTransform((0,self.command.pose.position.y,0),tf.transformations.quaternion_from_euler(0,0,0),rospy.Time.now(),'/robot_y_cmd','/robot_static_cmd')
    #publish transform from y motion to x motion
    self.br.sendTransform((self.command.pose.position.x,0,0),tf.transformations.quaternion_from_euler(0,0,0),rospy.Time.now(),'/robot_x_cmd','/robot_y_cmd')
    #publish transform from x motion to yaw motion
    self.br.sendTransform((0,0,0),tf.transformations.quaternion_from_euler(0,0,self.yawcommand),rospy.Time.now(),'/robot_yaw_cmd','/robot_x_cmd')
    #publish transform from yaw motion to z motion
    self.br.sendTransform((0,0,self.command.pose.position.z-.929*.0254),tf.transformations.quaternion_from_euler(0,0,0),rospy.Time.now(),'/robot_z_cmd','/robot_yaw_cmd')
    #publish transform from z motion to tilt motion
    self.br.sendTransform((0,0,0),tf.transformations.quaternion_from_euler(0,-self.pitchcommand,0),rospy.Time.now(),'/robot_pitch_cmd','/robot_z_cmd')

#self.command.pose.position.z-.929*.0254


      
#main function
def main(args):
  rospy.init_node('gantry_node', anonymous=True)
  encreader = FishGantry()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"

if __name__ == '__main__':
    main(sys.argv) 