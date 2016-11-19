#!/usr/bin/env python
import rospy
import timeit
import pygame
from kobuki_msgs.msg import BumperEvent
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import numpy as np
import glob

class Ctrl_Motion:
    #constructor
    def __init__(self):
        rospy.init_node('listener', anonymous=True)
        rospy.loginfo('hello kobuki')
        self._init_odometry_reset()
        self._init_params()	# initialize params
        self._init_pubsub()	# initialize pub & sub
        pygame.init()
        pygame.display.set_mode()

    def _init_params(self):
        self.twist = Twist()
        self.imu = 0
        #self.last_odom = 0
        self.odom = 0
        #self.odom_count = 0	# Odometry Information
        #self.distance = 1	# reference distance
        #self.bumper = 0	# bumper
        #self.state = 0 	# bumper state
        #self.sound = 0
        #self.freq = 10
        #self.rate = rospy.Rate(self.freq)
        self.twist.linear.x = 0
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = 0
        self.roll, self.pitch, self.yaw=[0,0,0]
        self.odomx=0
        self.odomy=0
        self.bumper0 = { BumperEvent.LEFT:0, BumperEvent.CENTER:1, BumperEvent.RIGHT:2, } 
        self.bumper1 = { BumperEvent.RELEASED:'Released', BumperEvent.PRESSED:'Pressed', }
        self.bumperS = [ 'Released',  'Released',  'Released', ]
        self.thetar=0;
        self.k_1=12
        self.k_2=10.5
        self.k_3=10.5
        self.scale=0.55
        self.t0=rospy.get_time()
        self.RunFlag=0
        self.fw=open('plot.txt','w')
        self.fw.truncate()

    

    def _init_odometry_reset(self):
        self.pub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty)
        self.pub.publish(Empty())
        print 'published'


    def _init_pubsub(self):
        self.cmd_vel_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.OdomInfoCallback)
        self.bumper_event_sub = rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, self.BumperEventCallback)
        #self.imu_sub = rospy.Subscriber("mobile_base/sensors/imu_data", Imu, self.ImuInfoCallback)
        #self.sound_sub = rospy.Subscriber("/mobile_base/commands/sound", Sound, self.SoundInfoCallback)


    def BumperEventCallback(self,BumperData):
	    self.bumperS[self.bumper0[BumperData.bumper]]=self.bumper1[BumperData.state]


    #def ImuInfoCallback(self,ImuData):
	#quat = ImuData.orientation
	#self.q = [quat.x, quat.y, quat.z, quat.w]
	#self.roll, self.pitch, self.yaw = euler_from_quaternion(self.q)
	#print yaw
	#print data
	#print '{0: >+7.2f}'.format(ImuData.linear_acceleration.x)+' '+ '{0: >+7.2f}'.format(ImuData.angular_velocity.z)
	#rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    def OdomInfoCallback(self,OdomData):
        self.odomx = OdomData.pose.pose.position.x
        self.odomy = OdomData.pose.pose.position.y
        quat = OdomData.pose.pose.orientation
        q = [quat.x, quat.y, quat.z, quat.w]
        self.roll, self.pitch, self.yaw = euler_from_quaternion(q)

    def MotionCtrl(self):
        #print self.yaw
        #self.twist.linear.x = 0
        for BumperEvent in self.bumperS:
            if BumperEvent=='Pressed':
            	    self.twist.angular.z=0;
                    self.twist.linear.x=0;
                    self.RunFlag=0
        for event in pygame.event.get():
            #if event.type == pygame.QUIT:
            #    pygame.exit() #if sys is imported
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    self.twist.linear.x = self.twist.linear.x+0.1;
                    print("Hey, you pressed the key, UP!")
                    print self.twist.linear.x
                if event.key == pygame.K_DOWN:
                    print("Hey, you pressed the key, DOWN!")
                    self.twist.linear.x = self.twist.linear.x-0.1;
                    print self.twist.linear.x
                if event.key == pygame.K_LEFT:
                    print("Hey, you pressed the key, LEFT!")
                    self.twist.angular.z = self.twist.angular.z+0.4;
                    print self.twist.angular.z
                if event.key == pygame.K_RIGHT:
                    print("Hey, you pressed the key, RIGHT!")
                    self.twist.angular.z= self.twist.angular.z-0.4;
                    print self.twist.angular.z
                if event.key == pygame.K_SPACE:
                    self.twist.angular.z=0;
                    self.twist.linear.x=0;
                    self.t0=rospy.get_time()
                    self.RunFlag=1-self.RunFlag
                    print("Fully stop!!")
                if event.key == pygame.K_q:
                    print("Pygame exit!")
                    pygame.exit()
        toc = rospy.get_time()-self.t0
        if self.RunFlag==1:
            xr=self.scale*toc
            dxr=self.scale
            yr=0.2*np.sin(self.scale*toc)
            dyr=0.2*self.scale*np.cos(self.scale*toc)
            thetar=np.arctan(dyr/dxr)
            dthetar=(1/(1+np.power(0.2*np.cos(self.scale*toc),2))*(-0.2*np.sin(self.scale*toc)*self.scale))
            wr=dthetar
            vr=dxr/np.cos(thetar)
            qrdot=np.array([[dxr],[dyr],[dthetar]])
            qr=np.array([[xr],[yr],[thetar]])
            q=np.array([[self.odomx],[self.odomy],[self.yaw]])
            ctheta=np.cos(self.yaw)
            stheta=np.sin(self.yaw)
            Te=np.array([[ctheta,-stheta,0],[-stheta,ctheta,0],[0,0,1]])
            e=np.dot(Te, qr-q)
            self.fw.write(str(toc)+'   '+str(qr[0][0])+' '+str(qr[1][0])+' '+str(qr[2][0])+'    '+str(q[0][0])+' '+str(q[1][0])+' '+str(q[2][0])+' '+str(self.twist.linear.x)+' '+str( self.twist.angular.z)+'\n')
            self.twist.linear.x=vr*np.cos(e[2][0])+self.k_1*e[0][0]
            self.twist.angular.z=wr+self.k_2*vr*e[1][0]+self.k_3*vr*np.sin(e[2][0])
            #self.fw.write(str(toc)+'   '+str(qr[0][0])+' '+str(qr[1][0])+' '+str(qr[2][0])+'    '+str(q[0][0])+' '+str(q[1][0])+' '+str(q[2][0])+' '+str(self.twist.linear.x)+' '+str( self.twist.angular.z)+'\n')
            self.cmd_vel_pub.publish(self.twist)
        #print self.RunFlag
        print "Current angle is:"+'{0:+.4f}'.format(self.yaw)+'   '+"Current postion is:"+'{0:+.4f}'.format(self.odomx)+' '+'{0:+.4f}'.format(self.odomy)

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaneously.
    #rospy.init_node('listener', anonymous=True)
    #rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, callback)
    #rospy.Subscriber("/mobile_base/sensors/imu_data", Imu, Imucallback)
    # spin() simply keeps python from exiting until this node is stopped
    trans_obj =Ctrl_Motion()
    tic = rospy.get_time()
    while not rospy.is_shutdown():
        trans_obj.MotionCtrl() # odom
	#test_trans_obj.ShowBumperEventInfo()
	#test_trans_obj.ShowImuInfo()	
	#test_trans.obj.ShowSoundInfo()
    rospy.spin()



if __name__ == '__main__':
    #cmd_vel_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist)
    #twist = Twist()
    listener()
