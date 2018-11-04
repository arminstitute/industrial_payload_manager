#!/usr/bin/env python

import rospy
import unittest
from industrial_payload_manager.msg import Payload, PayloadTarget, PayloadArray, \
    GripperTarget, ArucoGridboardWithPose, LinkMarkers
import numpy as np

class TestUrdfToPayload(unittest.TestCase):
    
    def test_urdf_to_payload(self):
        
        payload_array_msg_expected = rospy.wait_for_message("expected_payload", PayloadArray, timeout=5)                
        payload_array_msg = rospy.wait_for_message("payload", PayloadArray, timeout=5)
        self._assert_payload_array_equal(payload_array_msg, payload_array_msg_expected)
    
    def _assert_payload_array_equal(self, msg1, msg2):
        self.assertEqual(len(msg1.payloads), len(msg2.payloads))
        for p1, p2 in zip(msg1.payloads, msg2.payloads):
            self._assert_payload_equal(p1, p2)
        for t1, t2 in zip(msg1.payload_targets, msg2.payload_targets):
            self._assert_payload_target_equal(t1, t2)
        for l1, l2 in zip(msg1.link_markers, msg2.link_markers):
            self._assert_link_markers_equal(l1, l2)
        self.assertEqual(msg1.delete_payloads, msg2.delete_payloads)       
    
    def _assert_payload_equal(self, p1, p2):
        self.assertEqual(p1.header.frame_id, p2.header.frame_id)
        self.assertEqual(p1.name, p2.name)
        self._assert_pose_almost_equal(p1.pose, p2.pose)
        for t1, t2 in zip(p1.gripper_targets, p2.gripper_targets):
            self._assert_gripper_target_equal(t1, t2)
        self._assert_payload_geometry_equal(p1.visual_geometry, p2.visual_geometry)
        self._assert_payload_geometry_equal(p1.collision_geometry, p2.collision_geometry)
        
        for m1, m2 in zip(p1.markers, p2.markers):
            self._assert_aruco_gridboard_with_pose_equal(m1, m2)
        
        #TODO: assert inertia is equal
        
        self.assertAlmostEqual(p1.confidence, p2.confidence)
            
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
    
    def _assert_gripper_target_equal(self, t1, t2):
        self.assertEqual(t1.header.frame_id, t2.header.frame_id)
        self.assertEqual(t1.name, t2.name)
        self._assert_pose_almost_equal(t1.pose, t2.pose)
        np.testing.assert_allclose(t1.pickup_ft_threshold, t2.pickup_ft_threshold)
        np.testing.assert_allclose(t1.place_ft_threshold, t2.place_ft_threshold)
        
    def _assert_payload_geometry_equal(self, g1, g2):
        self.assertEqual(g1.primitives, g2.primitives)
        self._assert_pose_almost_equal(g1.primitive_poses, g2.primitive_poses)
        self.assertEqual(g1.primitive_colors, g2.primitive_colors)
        
        self.assertEqual(g1.mesh_resources, g2.mesh_resources)
        self._assert_pose_almost_equal(g1.mesh_poses, g2.mesh_poses)
        self.assertEqual(g1.mesh_scales, g2.mesh_scales)
        self.assertEqual(g1.mesh_colors, g2.mesh_colors)
        
    def _assert_aruco_gridboard_with_pose_equal(self, m1, m2):
        self.assertEqual(m1.header.frame_id, m2.header.frame_id)
        self.assertEqual(m1.name, m2.name)
        self._assert_pose_almost_equal(m1.pose, m2.pose)
        self.assertEqual(m1.marker, m2.marker)        
    
    def _assert_payload_target_equal(self, t1, t2):
        self.assertEqual(t1.header.frame_id, t2.header.frame_id)
        self.assertEqual(t1.name, t2.name)
        self._assert_pose_almost_equal(t1.pose, t2.pose)
        self.assertAlmostEqual(t1.confidence, t2.confidence)
        
    def _assert_link_markers_equal(self, l1, l2):
        self.assertEqual(l1.header.frame_id, l2.header.frame_id)        
        for m1, m2 in zip(l1.markers, l2.markers):
            self._assert_aruco_gridboard_with_pose_equal(m1, m2)
    
if __name__ == "__main__":

    rospy.init_node("test_urdf_to_payload", anonymous=True)
    
    import rostest
    rostest.rosrun("test_urdf_to_payload", "TestUrdfToPayload", TestUrdfToPayload)
    
    
    



