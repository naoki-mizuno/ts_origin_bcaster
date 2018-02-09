#!/usr/bin/env python2

import tf2_ros
import tf.transformations
from geometry_msgs.msg import Transform, Point
import numpy as np


class TFUtils:
    @staticmethod
    def tf_from_3_points(p1, p2, p3):
        """
        Creates an axis from three points. The axis is such that v12 (the
        vector from p1 to p2) is the x-axis, and the y-axis being
        on the plane which has p1, p2, and p3 and also perpendicular to v12.
        z-axis is the cross product of v12 and v13.

            * p3
           /
          /
         /
        *--------- * p2
        p1

        :param p1: [Point|array] point to be used as the origin
        :param p2: [Point|array] point to be used for the x-axis
        :param p3: [Point|array] point to be used to get the z-axis
        :return: [geometry_msgs/Transform] the transform
        """
        if type(p1) is Point:
            p1 = np.array([p1.x, p1.y, p1.z])
        else:
            p1 = np.array(p1)
        if type(p2) is Point:
            p2 = np.array([p2.x, p2.y, p2.z])
        else:
            p2 = np.array(p2)
        if type(p3) is Point:
            p3 = np.array([p3.x, p3.y, p3.z])
        else:
            p3 = np.array(p3)
        # Vector from p1 to p2 (X axis)
        v_x = p2 - p1
        v_x = v_x / np.linalg.norm(v_x)
        # Vector from p1 to p3
        v13 = p3 - p1
        v_z = np.cross(v_x, v13)
        v_z = v_z / np.linalg.norm(v_z)
        v_y = np.cross(v_z, v_x)
        v_y = v_y / np.linalg.norm(v_y)
        mat = tf.transformations.identity_matrix()
        mat[:3, :3] = np.column_stack((v_x, v_y, v_z))
        q = tf.transformations.quaternion_from_matrix(mat)

        transform = Transform()
        transform.translation.x = p1[0]
        transform.translation.y = p1[1]
        transform.translation.z = p1[2]
        transform.rotation.x = q[0]
        transform.rotation.y = q[1]
        transform.rotation.z = q[2]
        transform.rotation.w = q[3]
        return transform

    @staticmethod
    def add_inverse(a, b):
        """
        Applies the transform a * inv(b)
        :param a: [Transform|array] a transform
        :param b: [Transform|array] a transform
        :return: [Transform|array] a * inv(b)
        """
        # TF2 -> TF1
        if type(a) is Transform:
            tf_a = [
                [a.translation.x, a.translation.y, a.translation.z],
                [a.rotation.x, a.rotation.y, a.rotation.z, a.rotation.w]
            ]
        else:
            tf_a = a
        # TF2 -> TF1
        if type(b) is Transform:
            tf_b = [
                [b.translation.x, b.translation.y, b.translation.z],
                [b.rotation.x, b.rotation.y, b.rotation.z, b.rotation.w]
            ]
        else:
            tf_b = b

        a_trs = tf.transformations.translation_matrix(tf_a[0])
        a_qtn = tf.transformations.quaternion_matrix(tf_a[1])
        a_mat = tf.transformations.concatenate_matrices(a_trs, a_qtn)
        b_trs = tf.transformations.translation_matrix(tf_b[0])
        b_qtn = tf.transformations.quaternion_matrix(tf_b[1])
        b_mat = tf.transformations.concatenate_matrices(b_trs, b_qtn)
        tf_mat = np.dot(a_mat, tf.transformations.inverse_matrix(b_mat))

        if type(a) is Transform and type(b) is Transform:
            tr = tf.transformations.translation_from_matrix(tf_mat)
            rt = tf.transformations.quaternion_from_matrix(tf_mat)
            msg = Transform()
            msg.translation.x = tr[0]
            msg.translation.y = tr[1]
            msg.translation.z = tr[2]
            msg.rotation.x = rt[0]
            msg.rotation.y = rt[1]
            msg.rotation.z = rt[2]
            msg.rotation.w = rt[3]
            return msg
        else:
            return tf_mat

