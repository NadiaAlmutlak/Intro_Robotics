#!/usr/bin/env python

import numpy
import geometry_msgs.msg
import rospy
from sensor_msgs.msg import JointState
import tf
import tf.msg
from urdf_parser_py.urdf import URDF

"""This function will transform a 4x4 transformation matrix T into a ros message 
which can be published. In addition to the transform itself, the message
also specifies who is related by this transform, the parent and the child.
It is an optional function which you may use as you see fit."""


def convert_to_message(T, child, parent):
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = parent
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = child
    translation = tf.transformations.translation_from_matrix(T)
    rotation = tf.transformations.quaternion_from_matrix(T)
    t.transform.translation.x = translation[0]
    t.transform.translation.y = translation[1]
    t.transform.translation.z = translation[2]
    t.transform.rotation.x = rotation[0]
    t.transform.rotation.y = rotation[1]
    t.transform.rotation.z = rotation[2]
    t.transform.rotation.w = rotation[3]
    return t


# Our main class for computing Forward Kinematics
class ForwardKinematics(object):

    # Initialization
    def __init__(self):
        """Announces that it will publish forward kinematics results, in the form of tfMessages.
        "tf" stands for "transform library", it's ROS's way of communicating around information
        about where things are in the world"""
        self.pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage, queue_size=1)

        # Loads the robot model, which contains the robot's kinematics information
        self.robot = URDF.from_parameter_server()

        # Subscribes to information about what the current joint values are.
        rospy.Subscriber("joint_states", JointState, self.callback)

    """This function is called every time the robot publishes its joint values. You must use
    the information you get to compute forward kinematics.

    The callback you write should iterate through the entire robot chain, and publish 
    the transform for each link you find.
    """

    # OH notes: What the callback function is supposed to do is iterate through all the links and
    # pretty much repeat all we did in assignment 1 through the function itself.
    # You shouldn't be setting the transforms between jonits and links manually, the call back
    # function should do it for you.

    def callback(self, joint_values):
        # print joint_values # brings out the elements in the URDF
        robot = URDF.from_parameter_server()
        print joint_values.name

        link_name = self.robot.get_root()  # gets the root link from the robot simulation that we started up
        # print link_name
        # creates an empty list which we'll append to later
        link_names = []
        joints = []

        # We need to create an iterative process that generates the links and joints
        # to publish transforms
        while True:
            # print self.robot.child_map
            # This lets us know if the link (transform) exists between a parent and child
            if link_name not in self.robot.child_map:
                # print link_name
                break

                # We can only deal with 1-to-1 parent child cofigurations
            if len(self.robot.child_map[link_name]) != 1:
                rospy.logerror("Too many children");
                # print self.robot.child_map[link_name]
                # I was getting too many bugs so I wanted to know what was bugging up http://wiki.ros.org/rospy/Overview/Logging
                break
                # At this point, we should've eliminated any problematic parent/child relationships
            # and now we can link them together
            (next_joint_name, next_link_name) = self.robot.child_map[link_name][0]
            # print (next_joint_name,next_link_name)
            # print next_joint_name
            # print self.robot.joint_map
            # presents real joint
            if next_joint_name not in self.robot.joint_map:
                rospy.logerror("Joint error")
                break;
            joints.append(self.robot.joint_map[next_joint_name])

            link_names.append(next_link_name)
            # print link_names
            # print joints
            link_name = next_link_name

        all_transforms = self.compute_transforms(link_names, joints, joint_values)
        # print all_transforms
        self.pub_tf.publish(all_transforms)

        # we might have to set a joint manually, then arrange UR5 values man



        # we might have to set a joint manually, then arrange UR5 values man

    def compute_transforms(self, link_names, joints, joint_values):
        all_transforms = tf.msg.tfMessage()
        T = tf.transformations.identity_matrix()
        i = 0

        for link in link_names[1:]:
            # print link_names

            if joints[i].type == 'fixed':
                T = tf.transformations.translation_matrix(joints[0].origin.xyz)
                all_transforms.transforms.append(convert_to_message(T, link, link_names[0]))
                i += 1

            else:  # revolute
                # print joint_values
                index_of_q_i = joint_values.name.index(joints[i].name)
                q_i = joint_values.position[index_of_q_i]

                # rotation_matrix = tf.transformations.rotation_matrix(q_i, joints[i].axis)
                rotation_mat = tf.transformations.quaternion_matrix(
                    tf.transformations.quaternion_about_axis(q_i, joints[i].axis))
                translation_matrix = tf.transformations.translation_matrix(joints[i].origin.xyz)
                T_WL_CF_i = tf.transformations.concatenate_matrices(translation_matrix, rotation_mat)
                T = tf.transformations.concatenate_matrices(T, T_WL_CF_i)
                all_transforms.transforms.append(convert_to_message(T, link, link_names[0]))
                i += 1

            return all_transforms


if __name__ == '__main__':
    rospy.init_node('fwk', anonymous=True)
    fwk = ForwardKinematics()
    rospy.spin()





                # origin=joints[i].origin.xyz
                # rot=joints[i].origin.rpy
                # rot_matrix=tf.transformations.euler_matrix(rot[0],rot[1],rot[2])
                # trans=tf.transformations.translation_matrix(origin)
                # q_mat=tf.transformations.quaternion_about_axis(q_i,joints[i].axis)
                # rotation_transform=tf.transformations.quaternion_matrix(q_mat)
                # full_rotation=numpy.dot(trans,rot_matrix)
                # full_transform=tf.transformations.concatenate_matrices(full_rotation,rotation_transform)
                # all_transforms.transforms.append(convert_to_message(full_transform, link, link_names[0]))
                # i += 1
