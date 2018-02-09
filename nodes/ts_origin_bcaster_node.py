#!/usr/bin/env python2

import rospy
from ts_origin_bcaster.bcaster import Bcaster

rospy.init_node('ts_origin_bcaster_node')

b = Bcaster()

rospy.spin()
