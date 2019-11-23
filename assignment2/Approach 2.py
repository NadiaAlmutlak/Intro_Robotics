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
    ## REALLY REALLY REALLY IMPORTANT; DONT FORGET TO DO THE TRANSFORMS IN ORDER!!

    def callback(self, joint_values):

        # Pulls out information from URDF and pumps out the information we might need later
        robot = URDF.from_parameter_server()
        root = robot.get_root()
        final_link = root

        # Loops through to create a chain of the links and joints
        while True:
            try:
                (_, final_link) = robot.child_map[final_link][0]
            except:
                break

        # use get_chain to find the entire joint list and link list, quicker way : http://docs.ros.org/kinetic/api/urdfdom_py/html/classurdf__parser__py_1_1urdf_1_1Robot.html#a0c0c60eb94be9df6a8051e3fab668bf0
        jointList = robot.get_chain(root, final_link, joints=True, links=False, fixed=True)
        linkList = robot.get_chain(root, final_link, joints=False, links=True, fixed=True)
        # print jointList
        # print linkList

        transforms = self.compute_transforms(linkList, jointList, root, joint_values)
        self.pub_tf.publish(transforms)

    def compute_transforms(self, linkList, jointList, root, joint_values):

        '''we need to set up fixed joints for the robots. One of the issues I kept running into is that
        kuka only has 1 fixed joint whereas UR5 has 2.  '''

        if self.robot.name == "lwr":
            fixedJointName1 = jointList[0]  # worldjoint
            jointList = jointList[1:]

        elif self.robot.name == "ur5":
            fixedJointName1 = jointList[0]  # world joint
            fixedJointName2 = jointList[-1]  # ee_joint
            jointList = jointList[1:-1]

        # calcualte the transformation for the first fixed joint

        fixedJoint1 = self.robot.joint_map[fixedJointName1]
        T_link_origin = tf.transformations.translation_matrix(fixedJoint1.origin.xyz)
        T_joint_origin = tf.transformations.rotation_matrix(0, (0, 1, 0))
        T_origin = numpy.dot(T_link_origin, T_joint_origin)
        (_, child) = self.robot.child_map[root][0]
        TF_converter = convert_to_message(T_origin, child, root)
        linkList = linkList[1:]
        Transform = [TF_converter]
        # ID_mat = tf.transformations.identity_matrix() causes the joints to come apart
        T = [T_origin]
        '''we need to be able to parse using the joint names and positions'''

        jointNames = joint_values.name
        jointAngles = joint_values.position
        # print jointNames
        # print jointAngles

        # use loop to calculate the transforamtion for the rest of the joints
        for i in range(len(jointNames)):
            # print type(jointNames)

            '''Needed values'''
            jointName = jointList[i]
            parent = root
            child = linkList[i + 1]
            joint = self.robot.joint_map[jointName]
            rot_origin = joint.origin.rpy  # Kuka might not need RPY but will keep it just in case
            trans_origin = joint.origin.xyz
            index_of_qi = jointNames.index(jointName)
            q_i = jointAngles[index_of_qi]

            '''computing the transforms- first we generate the rotation transform, the translation transform and the rotation matrix
            then, in order, we dot the transltion (xyz matrix) and rotation (RPY matrix), then we dot that reult with the transformations 
            we get from the joints information.'''

            Tf_rotation = tf.transformations.euler_matrix(rot_origin[0], rot_origin[1], rot_origin[2])
            T_link = tf.transformations.translation_matrix(trans_origin)
            T_joint = tf.transformations.rotation_matrix(q_i, joint.axis)
            T.append(numpy.dot(numpy.dot(T_link, Tf_rotation), T_joint))

            ''' As you can tell I just realized I can comment with three ('') but anyways, here we computer the final transform by
             doting the identity matrix and our result which we appended to T'''
            final_transform = tf.transformations.identity_matrix()
            for l in xrange(numpy.shape(T)[0]):
                final_transform = numpy.dot(final_transform, T[l])
            Transform.append(convert_to_message(final_transform, child, parent))

        '''I was initially trying to make one loop work for both kuka and ur5 but I could't really figure it out so I just made an condition'''
        if robot.name == "ur5":

            '''needed values'''
            fixedJoint2 = self.robot.joint_map[fixedJointName2]
            rot_axis = fixedJoint2.origin.rpy
            trans_axis = fixedJoint2.origin.xyz

            '''computing the transforms- first we generate the rotation transform. However, unlink with te other case, we need to take into consideration
            the end links'''

            Tf_rotation = tf.transformations.euler_matrix(rot_axis[0], rot_axis[1], rot_axis[2])
            T_le = tf.transformations.translation_matrix(trans_axis)
            T_je = tf.transformations.rotation_matrix(0, (0, 1, 0))
            T.append(numpy.dot(T_le, numpy.dot(Tf_rotation, T_je)))

            '''we calculate the final transform similar to the other case.'''

            final_transform = tf.transformations.identity_matrix()
            for l in xrange(numpy.shape(T)[0]):
                final_transform = numpy.dot(final_transform, T[l])
            Transform.append(convert_to_message(final_transform, linkList[-1], root))

        return Transform
        # use publisher to publish all transformations


if __name__ == '__main__':
    rospy.init_node('fwk', anonymous=True)
    fwk = ForwardKinematics()
    rospy.spin()

