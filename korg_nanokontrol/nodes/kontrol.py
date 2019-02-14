#!/usr/bin/env python
#
# joystick input driver for Korg NanoKontrol input device
#
# Author: Austin Hendrix
# Author: Allison Thackston

import rospy
import sys
from korg_nanokontrol.korg import KorgNanoKontrol

if __name__ == '__main__':
    # start node
    rospy.init_node('kontrol')
    if len(sys.argv) > 1:
        input_dev = int(sys.argv[1])
        rospy.set_param("~input_dev", input_dev)
    try:
        kontrol = KorgNanoKontrol()
        rospy.on_shutdown(kontrol.finish)

        while not rospy.is_shutdown():
            kontrol.update()
            rospy.sleep(0.1)  # 10Hz maximum input

    except rospy.ROSInterruptException:
        pass
    except:
        rospy.logerr(sys.exc_info()[0])
