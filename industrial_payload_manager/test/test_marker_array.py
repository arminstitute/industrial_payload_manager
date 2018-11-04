#!/usr/bin/env python

import rospy
import unittest
from visualization_msgs.msg import MarkerArray
import numpy as np

class TestMarkerArray(unittest.TestCase):
    
    def test_marker_array(self):
        
        expected_marker_array = rospy.wait_for_message("/expected_payload_marker_array", MarkerArray, timeout=10)
        marker_array = rospy.wait_for_message("payload_marker_array", MarkerArray, timeout=10)
        
        markers=dict()
        expected_markers=dict()
        
        for m in marker_array.markers:
            markers[(m.ns, m.id)] = m
            
        for m in expected_marker_array.markers:
            expected_markers[(m.ns, m.id)] = m
        
        for k in markers:
            self._assert_marker_equal(markers[k], expected_markers[k])
        
    def _assert_marker_equal(self, m1, m2):
        self.assertEqual(m1.header.frame_id, m2.header.frame_id)
        self.assertEqual(m1.ns, m2.ns)
        self.assertEqual(m1.id, m2.id)
        self.assertEqual(m1.type, m2.type)
        self.assertEqual(m1.action, m2.action)
        self._assert_pose_almost_equal(m1.pose, m2.pose)
        self.assertEqual(m1.scale, m2.scale)
        self.assertEqual(m1.color, m2.color)
        self.assertEqual(m1.lifetime, m2.lifetime)
        self.assertEqual(m1.frame_locked, m2.frame_locked)
        self.assertEqual(m1.points, m2.points)
        self.assertEqual(m1.colors, m2.colors)
        self.assertEqual(m1.text, m2.text)
        self.assertEqual(m1.mesh_resource, m2.mesh_resource)
        self.assertEqual(m1.mesh_use_embedded_materials, m2.mesh_use_embedded_materials)
    
    def _assert_pose_almost_equal(self, pose1, pose2):
        if isinstance(pose1, list):
            for p1, p2 in zip(pose1, pose2):
                self._assert_pose_almost_equal(p1, p2)
            return
        np.testing.assert_allclose([pose1.position.x, pose1.position.y, pose1.position.z], \
                                    [pose2.position.x, pose2.position.y, pose2.position.z])
        np.testing.assert_allclose([pose1.orientation.x, pose1.orientation.y, \
                                pose1.orientation.z, pose1.orientation.w], \
                               [pose2.orientation.x, pose2.orientation.y, \
                                pose2.orientation.z, pose2.orientation.w] )
    
    
    
if __name__ == "__main__":

    rospy.init_node("test_marker_array", anonymous=True)
    
    import rostest
    rostest.rosrun("test_marker_array", "TestMarkerArray", TestMarkerArray)
