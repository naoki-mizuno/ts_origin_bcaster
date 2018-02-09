#!/usr/bin/env python2

import rospy
from ts_origin_bcaster.msg import GroundRef


rospy.init_node('ts_origin_provider_node')

gnss_gref = GroundRef()
gnss_gref.p1.x = rospy.get_param('~gnss/p1/x')
gnss_gref.p1.y = rospy.get_param('~gnss/p1/y')
gnss_gref.p1.z = rospy.get_param('~gnss/p1/z')
gnss_gref.p2.x = rospy.get_param('~gnss/p2/x')
gnss_gref.p2.y = rospy.get_param('~gnss/p2/y')
gnss_gref.p2.z = rospy.get_param('~gnss/p2/z')
gnss_gref.p3.x = rospy.get_param('~gnss/p3/x')
gnss_gref.p3.y = rospy.get_param('~gnss/p3/y')
gnss_gref.p3.z = rospy.get_param('~gnss/p3/z')
gnss_gref.is_llh = rospy.get_param('~gnss/is_llh')

ts_gref = GroundRef()
ts_gref.p1.x = rospy.get_param('~ts/p1/x')
ts_gref.p1.y = rospy.get_param('~ts/p1/y')
ts_gref.p1.z = rospy.get_param('~ts/p1/z')
ts_gref.p2.x = rospy.get_param('~ts/p2/x')
ts_gref.p2.y = rospy.get_param('~ts/p2/y')
ts_gref.p2.z = rospy.get_param('~ts/p2/z')
ts_gref.p3.x = rospy.get_param('~ts/p3/x')
ts_gref.p3.y = rospy.get_param('~ts/p3/y')
ts_gref.p3.z = rospy.get_param('~ts/p3/z')
ts_gref.is_llh = rospy.get_param('~ts/is_llh')

gnss_pub = rospy.Publisher('gnss_gref', GroundRef, queue_size=1, latch=True)
ts_pub = rospy.Publisher('ts_gref', GroundRef, queue_size=1, latch=True)

gnss_pub.publish(gnss_gref)
ts_pub.publish(ts_gref)

rospy.spin()
