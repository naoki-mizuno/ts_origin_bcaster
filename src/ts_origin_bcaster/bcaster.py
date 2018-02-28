#!/usr/bin/env python2

import rospy
import tf2_ros
from geometry_msgs.msg import Point, TransformStamped
from ts_origin_bcaster.msg import GroundRef
from ts_origin_bcaster.llh2xyz import LLH2XYZ
from ts_origin_bcaster.tf_utils import TFUtils


class Bcaster:
    def __init__(self):
        self.tf_static = tf2_ros.StaticTransformBroadcaster()
        self.gps_origin_frame_id = rospy.get_param('~gps_origin_frame_id',
                                                   'gps_origin')
        self.ts_origin_frame_id = rospy.get_param('~ts_origin_frame_id',
                                                  'total_station')
        self.publish_gref = rospy.get_param('~publish_gref', False)
        self.src_epsg = rospy.get_param('~src_epsg', 4326)
        # Japan Plane Rectangular CS X (JGD2011)
        self.tgt_epsg = rospy.get_param('~tgt_epsg', 6678)
        self.converter = LLH2XYZ(src_epsg=self.src_epsg,
                                 tgt_epsg=self.tgt_epsg)
        # Ground reference coordinate values
        self.gnss_gref = None
        self.ts_gref = None

        self.gnss_sub = rospy.Subscriber('gnss_gref',
                                         GroundRef,
                                         self.gnss_cb,
                                         queue_size=1)
        self.ts_sub = rospy.Subscriber('ts_gref',
                                       GroundRef,
                                       self.ts_cb,
                                       queue_size=1)

    def gnss_cb(self, msg):
        self.gnss_gref = msg
        if msg.is_llh:
            self.gnss_gref.p1 = self.__convert_llh_point__(msg.p1)
            self.gnss_gref.p2 = self.__convert_llh_point__(msg.p2)
            self.gnss_gref.p3 = self.__convert_llh_point__(msg.p3)
        self.broadcast()

    def ts_cb(self, msg):
        self.ts_gref = msg
        # Super unlikely, but just in case TotalStation outputs points in LLH
        if msg.is_llh:
            self.ts_gref.p1 = self.__convert_llh_point__(msg.p1)
            self.ts_gref.p2 = self.__convert_llh_point__(msg.p2)
            self.ts_gref.p3 = self.__convert_llh_point__(msg.p3)
        self.broadcast()

    def broadcast(self):
        if self.gnss_gref is None or self.ts_gref is None:
            return

        gnss_tf = TFUtils.tf_from_3_points(self.gnss_gref.p1,
                                           self.gnss_gref.p2,
                                           self.gnss_gref.p3)
        ts_tf = TFUtils.tf_from_3_points(self.ts_gref.p1,
                                         self.ts_gref.p2,
                                         self.ts_gref.p3)
        tform_gnss = TransformStamped()
        tform_gnss.header.stamp = rospy.Time.now()
        tform_gnss.header.frame_id = self.gps_origin_frame_id
        tform_gnss.child_frame_id = 'gref_gnss'
        tform_gnss.transform = gnss_tf
        tform_ts = TransformStamped()
        tform_ts.header.stamp = rospy.Time.now()
        tform_ts.header.frame_id = self.ts_origin_frame_id
        tform_ts.child_frame_id = 'gref_ts'
        tform_ts.transform = ts_tf

        gnss_to_ts = TFUtils.add_inverse(gnss_tf, ts_tf)

        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.gps_origin_frame_id
        t.child_frame_id = self.ts_origin_frame_id
        t.transform = gnss_to_ts
        tforms = [t]
        if self.publish_gref:
            tforms.append(tform_gnss)
            tforms.append(tform_ts)
        self.tf_static.sendTransform(tforms)

    def __convert_llh_point__(self, llh_point):
        """
        Converts a point in lat, lon, height to xyz
        :param llh_point: Point with x being lat, y being lon, z being height
        :return: a geometry_msgs/Point with lat, lon, height, converted to
        x, y, z in the coordinate system specified as ROS parameters.
        """
        lat = llh_point.x
        lon = llh_point.y
        height = llh_point.z
        x, y, z = self.converter.convert(lat, lon, height)
        xyz_point = Point()
        xyz_point.x = x
        xyz_point.y = y
        xyz_point.z = z
        return xyz_point
