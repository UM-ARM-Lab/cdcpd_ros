#!/usr/bin/env python
from __future__ import print_function

import argparse
import numpy as np

import ros_np_multiarray as rnm
import ros_numpy
import rospy
import tf2_ros
from victor_hardware_interface_msgs.msg import Robotiq3FingerStatus
from cdcpd_ros.msg import Float32MultiArrayStamped
from tf.transformations import euler_from_quaternion


class PGI:

    def __init__(self):
        # make a TF listener
        parser = argparse.ArgumentParser()

        args = parser.parse_args()

        rospy.init_node('pub_gripper_info')

        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        config_pub = rospy.Publisher('kinect2_victor_head/qhd/gripper_config', Float32MultiArrayStamped, queue_size=10)
        dot_pub = rospy.Publisher('kinect2_victor_head/qhd/dot_config', Float32MultiArrayStamped, queue_size=10)

        self.repub_left_gripper_status = rospy.Publisher('left_arm/gripper_status_repub', Robotiq3FingerStatus, queue_size=10)
        self.repub_right_gripper_status = rospy.Publisher('right_arm/gripper_status_repub', Robotiq3FingerStatus, queue_size=10)

        sub_left_gripper_status = rospy.Subscriber('left_arm/gripper_status', Robotiq3FingerStatus, self.left_gripper_cb)
        sub_right_gripper_status = rospy.Subscriber('right_arm/gripper_status', Robotiq3FingerStatus, self.right_gripper_cb)

        # gripper TF names
        gripper_tf_names = []
        gripper_tf_names.append("left_tool_placeholder")
        gripper_tf_names.append("right_tool_placeholder")
        rospy.sleep(1)
        # at some fixed frequency, query TF for the gripper pose in camera rgbd frame?
        dt = 10
        rate = rospy.Rate(dt)
        previous_config_vectors = []
        while not rospy.is_shutdown():
            configs_list = []
            dots_list = []
            for i, gripper_frame in enumerate(gripper_tf_names):
                try:
                    from_rgbd_frame = "kinect2_victor_head_link"
                    gripper_transform = tfBuffer.lookup_transform(from_rgbd_frame, gripper_frame, rospy.Time())
                except Exception as e:
                    print(e)
                    rate.sleep()
                    continue

                transform_matrix = ros_numpy.numpify(gripper_transform.transform)
                current_position = ros_numpy.numpify(gripper_transform.transform.translation)
                q = [gripper_transform.transform.rotation.w,
                     gripper_transform.transform.rotation.x,
                     gripper_transform.transform.rotation.y,
                     gripper_transform.transform.rotation.z]
                current_euler_angles = np.array(euler_from_quaternion(q))
                current_config_vector = np.concatenate([current_position, current_euler_angles])
                config = transform_matrix.flatten()

                if len(previous_config_vectors) <= i:
                    print("adding gripper", i)
                    dot = np.zeros(6)
                    previous_config_vectors.append(current_config_vector)
                else:
                    dot = (current_config_vector - previous_config_vectors[i]) * dt
                previous_config_vectors[i] = current_config_vector
                configs_list.append(config)
                dots_list.append(dot)

            # convert tf result to the right arrays

            # publish the config, dot, and other messages
            config_msg = Float32MultiArrayStamped()
            dot_msg = Float32MultiArrayStamped()
            # ind_msg = Float32MultiArrayStamped()

            # set data
            config_msg.data = rnm.to_multiarray_f32(np.array(configs_list, dtype=np.float32))
            dot_msg.data = rnm.to_multiarray_f32(np.array(dots_list, dtype=np.float32))
            # ind_msg.data = rnm.to_multiarray_f32(ind_array, dtype=np.float32))

            # set stamp
            config_msg.header.stamp = rospy.Time.now()
            dot_msg.header.stamp = rospy.Time.now()
            # ind_msg.header.stamp = rospy.Time.now()

            config_pub.publish(config_msg)
            dot_pub.publish(dot_msg)
            # ind_pub.publish(ind_msg)
            print("publish one frame")
            rate.sleep()


    def left_gripper_cb(self, msg):
        msg.header.stamp = rospy.Time.now()
        self.repub_left_gripper_status.publish(msg)

    def right_gripper_cb(self, msg):
        msg.header.stamp = rospy.Time.now()
        self.repub_right_gripper_status.publish(msg)

if __name__ == '__main__':
    i = PGI()
