#!/usr/bin/env python  
import rospy
import tf
import tf2_ros
import geometry_msgs.msg
import numpy


def message_from_transform(T):  # This function uses the final Rres and Tres and applies them to the frame so I dont have to repeat it in each frame's code
    nt = geometry_msgs.msg.Transform()
    q = tf.transformations.quaternion_from_matrix(T)
    p = tf.transformations.translation_from_matrix(T)
    nt.rotation.x = q[0]  # this works on indexing the quaternion //
    nt.rotation.y = q[1]  # nt already contains the .transform so adding it causes an error.
    nt.rotation.z = q[2]
    nt.rotation.w = q[3]
    nt.translation.x = p[0]  # this works on indexing the matrix
    nt.translation.y = p[1]
    nt.translation.z = p[2]
    return nt


# for office hours-- is this a good way to stop repeating this for each transformation? can something like this work?
##sources- http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Transform.html , the ros message tutorials, ros message boards.


def publish_transforms():  # taken from tf2_example and repurposed

    # base frame to object frame transformations
    t1 = geometry_msgs.msg.TransformStamped()
    t1.header.stamp = rospy.Time.now()
    t1.header.frame_id = "base_frame"  # should create Bf--> Of
    t1.child_frame_id = "object_frame"
    rot1 = tf.transformations.quaternion_from_euler(0.64, 0.64, 0)
    trans1 = tf.transformations.translation_matrix([1.5, 0.8, 0.0])
    T1 = numpy.dot(tf.transformations.quaternion_matrix(rot1), trans1)
    # this dot product produces a matrix which contains Rres and Tres and can be directly used to plug into a transformation from matrix.
    t1.transform = message_from_transform(T1)  # send back resultant matrix to produce final coordinates
    br.sendTransform(t1)

    # base frame to robot frame transformations
    t2 = geometry_msgs.msg.TransformStamped()
    t2.header.stamp = rospy.Time.now()
    t2.header.frame_id = "base_frame"
    t2.child_frame_id = "robot_frame"  # should create Bf--> Rf
    rot2 = tf.transformations.quaternion_about_axis(1.5, (
    0, 1, 0))  # assumes radians works as the angle and rotates around y axis
    trans2 = tf.transformations.translation_matrix([0.0, 0.0, -2.0])  # results in negative translation
    T2 = numpy.dot(tf.transformations.quaternion_matrix(rot2), trans2)
    t2.transform = message_from_transform(T2)  # send back resultant matrix to produce final coordinates
    br.sendTransform(t2)

    t3 = geometry_msgs.msg.TransformStamped()
    t3.header.stamp = rospy.Time.now()
    t3.header.frame_id = "robot_frame"  # should create Rf--> Cf
    t3.child_frame_id = "camera_frame"
    trans3 = tf.transformations.translation_matrix([0.3, 0.0, 0.3])  # creates robot to camera translation
    bp = T1[:, 3]  # get bp (base to object trans)
    print bp
    # print bp
    rtc = tf.transformations.inverse_matrix(trans3)  # find vector - cp camera to object
    btr = tf.transformations.inverse_matrix(T2)  # finds base to robot inverse
    print rtc
    print btr
    cp = numpy.matmul(numpy.dot(rtc, btr), bp)  # multiplies the matrices for (rtc)^-1 (btr)^-1 (bp)=cp
    cp = cp[:3]  # removes extra column because cross product needs to have similar dimensions as x_ax
    # print numpy.dot(rtc,btr)
    # print bp
    # print cp
    # print numpy.array([1,0,0])
    x_ax = numpy.array([1, 0, 0])
    vector = numpy.cross(x_ax, cp)
    # print vector
    num = numpy.dot(x_ax, cp)
    den = numpy.linalg.norm(x_ax) * numpy.linalg.norm(cp)
    angle = numpy.arccos(num / den)  # find angle between x-axis and vector using dot product - see numpy documentation
    # print angle
    # apply rotation using angle
    rot3 = tf.transformations.rotation_matrix(angle, vector)
    T3 = numpy.dot(trans3, rot3)
    t3.transform = message_from_transform(T3)
    br.sendTransform(t3)


if __name__ == '__main__':
    rospy.init_node('solution')

    br = tf2_ros.TransformBroadcaster()
    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        publish_transforms()
        rospy.sleep(0.05)
