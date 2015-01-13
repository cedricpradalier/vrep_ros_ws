#!/usr/bin/env python
import roslib; roslib.load_manifest('wpa_cli')
import rospy
import subprocess
import re

from disk_monitor.msg import DiskStatus

rospy.init_node('disk_monitor')
pub = rospy.Publisher('~disk',DiskStatus,latch=True)
update_period = rospy.get_param("~update_period",10.0)
dev = rospy.get_param("~device","/dev/sda3")

while not rospy.is_shutdown():
    scan_results = subprocess.check_output(["df", "-m"]).split("\n")
    for line in scan_results:
        l = []
        line = line.strip()
        while len(line)>0:
            # print line
            l.append(line.split(" ")[0])
            line = (" ".join(line.split(" ")[1:])).strip()
        # print l
        if len(l)==0 or (l[0] != dev):
            continue
        state = DiskStatus()
        state.device = l[0]
        state.mounted = l[5]
        state.size_mb = int(l[1])
        state.used_mb = int(l[2])
        state.available_mb = int(l[3])
        # print state
        pub.publish(state)
    rospy.sleep(update_period)

