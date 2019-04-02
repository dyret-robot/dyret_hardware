#!/usr/bin/env python

import roslib
roslib.load_manifest('diagnostic_updater')
import rospy, rostopic
import diagnostic_updater
import diagnostic_msgs
import std_msgs
import math

from geometry_msgs.msg import PoseStamped

def checkMocap(stat):
    global lastTime

    n = len(r.times)
    mean = sum(r.times) / n
    rate = 1./mean if mean > 0. else 0

    if lastTime == r.msg_tn:
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Not receiving mocap data")
    elif rate < 80:
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "Receiving mocap data at <80hz ({0:.1f})".format(rate))
    else:
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK,"OK ({0:.0f})".format(rate))

    stat.add("Frequency", "{0:.1f}".format(rate))

    lastTime = r.msg_tn

    return stat

rospy.init_node("mocapMonitor")

updater = diagnostic_updater.Updater()
updater.setHardwareID("dyret")

r = rostopic.ROSTopicHz(100)
s = rospy.Subscriber('/dyret/sensor/pose', rospy.AnyMsg, r.callback_hz)

updater.add("mocap topic status", checkMocap)

lastTime = 0

while not rospy.is_shutdown():
    rospy.sleep(1)

    updater.update()


