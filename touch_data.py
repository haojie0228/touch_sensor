#!/usr/bin/env python3

from math import ceil
import rospy
from std_msgs.msg import Float32MultiArray


def callback(data):
    
    S1_CH_1 = data.data[0]
    S1_CH_3 = data.data[2]
    S1_CH_4 = data.data[3]
    S1_CH_5 = data.data[4]
    S1_CH_2 = data.data[1]
    S1_CH_6 = data.data[5]
    S2_CH_1 = data.data[6]
    S2_CH_2 = data.data[7]
    S2_CH_3 = data.data[8]
    S2_CH_4 = data.data[9]
    S2_CH_5 = data.data[10]
    S2_CH_6 = data.data[11]
    #rospy.loginfo(rospy.get_caller_id() + "S1_CH_1 is %s", S1_CH_1)
    rospy.loginfo(rospy.get_caller_id() + "S1_CH_2 is %s", S1_CH_2)
    rospy.loginfo(rospy.get_caller_id() + "S1_CH_3 is %s", S1_CH_3)
    rospy.loginfo(rospy.get_caller_id() + "S1_CH_4 is %s", S1_CH_4)
    rospy.loginfo(rospy.get_caller_id() + "S1_CH_5 is %s", S1_CH_5)
    #rospy.loginfo(rospy.get_caller_id() + "S1_CH_6 is %s", S1_CH_6)

    #rospy.loginfo(rospy.get_caller_id() + "S2_CH_1 is %s", S2_CH_1)
    rospy.loginfo(rospy.get_caller_id() + "S2_CH_2 is %s", S2_CH_2)
    rospy.loginfo(rospy.get_caller_id() + "S2_CH_3 is %s", S2_CH_3)
    rospy.loginfo(rospy.get_caller_id() + "S2_CH_4 is %s", S2_CH_4)
    rospy.loginfo(rospy.get_caller_id() + "S2_CH_5 is %s", S2_CH_5)
    #rospy.loginfo(rospy.get_caller_id() + "S2_CH_6 is %s", S2_CH_6)
    

def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("sensor_data", Float32MultiArray, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()