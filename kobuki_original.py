#!/usr/bin/env python
import rospy
import time
import pygame
from kobuki_msgs.msg import BumperEvent
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty


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
        self.freq = 5
        self.rate = rospy.Rate(self.freq)
        self.twist.linear.x = 0
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = 0
        self.roll, self.pitch, self.yaw=[0,0,0]
        self.odomx=0
        self.odomy=0


    def _init_odometry_reset(self):
        self.pub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty)
        self.pub.publish(Empty())
        print 'published'



    def _init_pubsub(self):
        self.cmd_vel_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.OdomInfoCallback)
        #self.bumper_event_sub = rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, self.BumperEventCallback)
        #self.imu_sub = rospy.Subscriber("mobile_base/sensors/imu_data", Imu, self.ImuInfoCallback)
        #self.sound_sub = rospy.Subscriber("/mobile_base/commands/sound", Sound, self.SoundInfoCallback)


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
                    print("Fully stop!!")
                if event.key == pygame.K_q:
                    print("Pygame exit!")
                    pygame.exit()
        self.cmd_vel_pub.publish(self.twist)
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
