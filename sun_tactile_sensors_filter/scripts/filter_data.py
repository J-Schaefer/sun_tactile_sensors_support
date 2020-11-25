#!/usr/bin/python

import rospy
import numpy as np
from scipy.signal import butter,filtfilt,freqz,lfilter_zi,lfilter,firwin,kaiserord
from geometry_msgs.msg import WrenchStamped

pub = None

def low_pass_filter(data):
    global z
    filtered_data, z= lfilter(b, 1, [data], zi=z)
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
    global b
    global z
    rospy.init_node('sun_finger_filter', anonymous=False)

    finger_no = rospy.get_param("~finger_number")
    fs = rospy.get_param("~samplerate")
    rospy.loginfo("Starting node for finger " + str(finger_no) + " with a sample rate of " + str(fs) + " Hz")

    cutoff = 100      # desired cutoff frequency of the filter, Hz ,      slightly higher than actual 1.2 Hz
    # The Nyquist rate of the signal.
    nyq_rate = fs / 2.0

    # The desired width of the transition from pass to stop,
    # relative to the Nyquist rate.  We'll design the filter
    # with a 5 Hz transition width.
    width = 5.0/nyq_rate

    # The desired attenuation in the stop band, in dB.
    ripple_db = 60.0

    # Compute the order and Kaiser parameter for the FIR filter.
    N, beta = kaiserord(ripple_db, width)

    # Get the filter coefficients
    # b = firwin(150,cutoff=cutoff, fs=fs)

    # Use firwin with a Kaiser window to create a lowpass FIR filter.
    b = firwin(N, cutoff/nyq_rate, window=('kaiser', beta))
    z = lfilter_zi(b, 1) # get initial conditions

    if finger_no == 0:
        topic_sub = "/wsg50/finger0/wrench"
        topic_pub = "/wsg50/finger0/wrench_filtered"
    elif finger_no == 1:
        topic_sub = "/wsg50/finger1/wrench"
        topic_pub = "/wsg50/finger1/wrench_filtered"
    else:
        rospy_error("Wrong finger number. I'm not doing anything.")
        return 1

    rospy.loginfo("Subscribing to " + topic_sub)
    rospy.Subscriber(topic_sub, WrenchStamped, throw_data)
    pub = rospy.Publisher(topic_pub, WrenchStamped, queue_size=10)
    rospy.spin()
    return 0

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
