#/usr/bin/python

import rospy
import numpy as np
from scipy.signal import butter,filtfilt,freqz,lfilter_zi,lfilter,firwin
from geometry_msgs.msg import WrenchStamped


# Filter requirements.
fs = 350.0       # sample rate, Hz
cutoff = 2      # desired cutoff frequency of the filter, Hz ,      slightly higher than actual 1.2 Hz

# Get the filter coefficients
b = firwin(150,cutoff=cutoff, fs=fs)
z = lfilter_zi(b, 1)

pub = None

def low_pass_filter(data):
    global z
    filtered_data, z = lfilter(b, 1, [data], zi=z)
    return filtered_data

def throw_data(data):
    global pub
    msg = WrenchStamped()
    msg = data
    msg.wrench.force.x = low_pass_filter(data.wrench.force.x)
    msg.wrench.force.y = low_pass_filter(data.wrench.force.y)
    msg.wrench.force.z = low_pass_filter(data.wrench.force.z)
    msg.wrench.torque.x = low_pass_filter(data.wrench.torque.x)
    msg.wrench.torque.y = low_pass_filter(data.wrench.torque.y)
    msg.wrench.torque.z = low_pass_filter(data.wrench.torque.z)
    pub.publish(msg)

def init():
    global pub
    rospy.loginfo()
    finger_no = rospy.get_param("/finger")
    rospy.init_node('sun_finger_filter', anonymous=True)

    if finger_no == 0:
        topic_sub = "/wsg50/finger0/wrench"
        topic_pub = "/wsg50/finger0/wrench_filtered"
    elif finger_no == 1:
        topic_sub = "/wsg50/finger1/wrench"
        topic_pub = "/wsg50/finger1/wrench_filtered"
    else:
        rospy_error("Wrong finger number. I'm not doing anything.")
        return 1

    rospy.Subscriber(topic_sub, WrenchStamped, throw_data)
    pub = rospy.Publisher(topic_pub, WrenchStamped, queue_size=10)
    rospy.spin()
    return 0

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
