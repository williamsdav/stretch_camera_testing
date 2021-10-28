#!/usr/bin/env python3
from __future__ import print_function

import numpy as np
import rospy
import tf

from std_msgs.msg import Float32

def move_camera():
    rospy.init_node('camera_tracking')

    pub = rospy.Publisher('camera_tilt', Float32, queue_size=10)
    tf_listener = tf.TransformListener()

    rospy.loginfo('checking transform')
    ab = tf_listener.lookupTransform('base_link', 'link_head_tilt', rospy.Time(0))
    ac = tf_listener.lookupTransform('base_link', 'link_gripper', rospy.Time(0))

    ab = np.array(ab[0])
    ac = np.array(ac[0])
    bc = np.add(-ab,ac)

    theta = np.arctan2(bc[2],-bc[1])

    while not rospy.is_shutdown():
        rospy.loginfo(theta)
        pub.publish(theta)