#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
import math

def commander():
    pub = rospy.Publisher('turntable_controller/command', Float64, queue_size=1)
    rospy.init_node('command_turntable', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        deg = raw_input('input angle in degrees: ')
        rad = float(deg)/ 180.0 * math.pi
        rospy.loginfo(deg)
        pub.publish(rad)
        rate.sleep()

if __name__ == '__main__':
    try:
        commander()
    except rospy.ROSInterruptException:
        pass