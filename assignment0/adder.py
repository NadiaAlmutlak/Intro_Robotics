
#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16
from assignment0 import TwoInt

def sum(data):
    pub = rospy.Publisher('/sum', Int16, queue_size=10)
    while not rospy.is_shutdown():



def adder():
    rospy.init_node('adder', anonymous=True)
    rospy.Subscriber("/num", TwoInt, sum)
       # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    adder()