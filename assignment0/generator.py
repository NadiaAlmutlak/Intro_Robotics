 #!/usr/bin/env python
import rospy
from std_msgs.msg import Int8
from assignment0.msg import TwoInt
from random import randint


def generate():
    pub = rospy.Publisher('/num', TwoInt, queue_size=10)
    rospy.init_node('TwoInt', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    num=TwoInt()

    while not rospy.is_shutdown():
        TwoInt.num1=[randint(0, 100) for p in range(0, 100)]
        TwoInt.num2=[randint(0, 100) for p in range(0, 100)]
        rospy.loginfo(num)
        pub.publish(num)
        rate.sleep()


if __name__ == '__main__':
    try:
        generate()
    except rospy.ROSInterruptException:
        pass
