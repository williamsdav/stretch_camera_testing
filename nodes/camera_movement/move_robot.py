#!/usr/bin/env python3
from __future__ import print_function

import numpy as np
import rospy
import tf

from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
import hello_helpers.hello_misc as hm

class MoveRobot(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        rospy.loginfo('initializing')

        self.joint_states = None
        self.camera_tilt = None

    def joint_state_callback(self,joint_states):
        self.joint_states = joint_states

    def camera_callback(self,camera_tilt):
        self.camera_tilt = camera_tilt
        rospy.loginfo(camera_tilt.data)

    def home(self):
        rospy.loginfo('homing arm')
        self.move_to_pose({'joint_lift':0.2, 'wrist_extension':0.01, 'joint_wrist_yaw':np.pi})

        rospy.loginfo('homing camera')
        self.move_to_pose({'joint_head_pan':0, 'joint_head_tilt':0})

    def move_camera(self):
        rospy.loginfo('finding wrist')

        self.move_to_pose({'joint_head_tilt':self.camera_tilt})
        rospy.sleep(1)

        # correcting and iterating through error values
        e = 1
        e_previous = 2
        k_p = 0.8

        n = 0
        while abs(e) > 0.01 and n <= 3:
            e_previous = e
            error = self.tf_listener.lookupTransform('camera_link','link_gripper',rospy.Time(0))
            e = error[0][1]

            if abs(e) > abs(e_previous):
                break

            correction = k_p * np.arctan2(error[0][1],error[0][0])
            self.move_to_pose({'joint_head_tilt':self.camera_tilt + correction})
            rospy.sleep(0.25)
            rospy.loginfo('error: %f' %e)

            n += 1

    def move_in_diamond(self):
        rospy.loginfo('moving')

        self.move_to_pose({'joint_head_pan':-np.pi/2})

        # establish parameters
        lift = [0.4,0.6,0.8,0.6,0.4]
        wrist = [0.3,0.5,0.3,0.1,0.3]
        yaw = [np.pi/2,0,np.pi/2,np.pi,np.pi/2]

        # create loop
        for i in range(5):
            self.move_to_pose({'joint_lift':lift[i], 'wrist_extension':wrist[i], 'joint_wrist_yaw':yaw[i]})
            self.move_camera()
            rospy.sleep(.25)

    def main(self):
        hm.HelloNode.main(self,'move_robot_node','move_robot_namespace',wait_for_first_pointcloud=False)
        self.tf_listener = tf.TransformListener()
        
        rospy.Subscriber('/stretch/joint_states', JointState, self.joint_state_callback)
        rospy.Subscriber('camera_tilt',Float32,self.camera_callback)

        while self.joint_states is None:
            pass

        self.home()
        self.move_in_diamond()
        self.home()

if __name__ == '__main__':
    move_robot = MoveRobot()
    move_robot.main()