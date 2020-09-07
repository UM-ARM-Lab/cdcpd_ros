#!/usr/bin/env python
from __future__ import print_function

import ros_np_multiarray as rnm
import numpy as np
from tf.transformations import euler_from_quaternion
import ros_numpy
from cdcpd_ros.msg import Float32MultiArrayStamped
import argparse
import rospy

import math
import tf2_ros

def main():
    # make a TF listener
    parser = argparse.ArugmentParser()

    args = parser.parse_args()

    rospy.init_node('pub_gripper_info')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    config_pub = rospy.Publisher('kinect2/qhd/gripper_config', cdcpd_ros.Float32MultiArrayStamped, queue_size=10)
    dot_pub = rospy.Publisher('kinect2/qhd/dot_config', cdcpd_ros.Float32MultiArrayStamped, queue_size=10)
    ind_pub = rospy.Publisher('kinect2/qhd/ind_config', cdcpd_ros.Float32MultiArrayStamped, queue_size=10)
    gripper_status_pub = rospy.Publisher('gripper_status', ???, queue_size=10)

    # gripper TF names
    gripper_tf_names = []
    gripper_tf_names.append("victor_left_tool_placeholder")
    gripper_tf_names.append("victor_right_tool_placeholder")

    # at some fixed frequency, query TF for the gripper pose in camera rgbd frame?
    rate = rospy.Rate(0.01)
    previous_config_vectors = []
    while not rospy.is_shutdown():
        configs_list = []
        dots_list = []
        for i, gripper_frame in enumerate(gripper_tf_names):
            try:
                from_rgbd_frame = "kinect2_victor_header_link"
                gripper_transform = tfBuffer.lookup_transform(from_rgbd_frame, gripper_frame, rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue

            transform_matrix = ros_numpy.numpify(gripper_transform.transform)
            current_position = gripper_transform.transform.translation
            current_euler_angles = euler_from_quaternion(gripper_transform.transform.rotation)
            current_config_vector = np.array([current_position + current_euler_angles]) # plus concats
            config = transform_matrix.flatten()

            if len(previous_config_vectors) == 0:
                dot = np.zeros(6)
            else:
                dot = current_config_vector - previous_config_vectors[i]
            previous_config_vector[i] = current_config_vector
            configs_list.append(config)
            dots_list.append(dot)

        # convert tf result to the right arrays

        # publish the config, dot, and ind messages
        config_msg = Float32MultiArrayStamped()
        dot_msg = Float32MultiArrayStamped()
        #ind_msg = Float32MultiArrayStamped()

        # gripper status
        gripper_status_msg = ???()
        gripper_status.

        # set data
        config_msg.data = rnm.to_multiarray_f32(np.array(configs_list), dtype=np.float32))
        dot_msg.data = rnm.to_multiarray_f32(np.array(dots_list), dtype=np.float32))
        #ind_msg.data = rnm.to_multiarray_f32(ind_array, dtype=np.float32))

        # set stamp
        config_msg.header.stamp = rospy.Time.now()
        dot_msg.header.stamp = rospy.Time.now()
        #ind_msg.header.stamp = rospy.Time.now()

        config_pub.publish(config_msg)
        dot_pub.publish(dot_msg)
        # ind_pub.publish(ind_msg)
        gripper_status_pub.publish(gripper_status_msg)

        rate.sleep()


if __name__ == '__main__':
    main()
