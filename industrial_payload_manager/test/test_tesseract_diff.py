#!/usr/bin/env python

import rospy
import unittest
from industrial_payload_manager.msg import Payload, PayloadTarget, PayloadArray, \
    GripperTarget, ArucoGridboardWithPose, LinkMarkers
from industrial_payload_manager.srv import UpdatePayloadPose, UpdatePayloadPoseRequest
from tesseract_msgs.msg import TesseractState
import numpy as np
import Queue
from industrial_payload_manager.payload_transform_listener import PayloadTransformListener
import general_robotics_toolbox as rox
import general_robotics_toolbox.ros_msg as rox_msg

class TestTesseractDiff(unittest.TestCase):
    
    def __init__(self, *args, **kwargs):
        super(TestTesseractDiff,self).__init__(*args, **kwargs)
        self.tf_listener=PayloadTransformListener()
            
    def test_tesseract_diff(self):
        
        expected_state_1 = rospy.wait_for_message("/expected_tesseract_diff_1", TesseractState, timeout=10)        
        state_1 = rospy.wait_for_message('/tesseract_diff', TesseractState, timeout=10)        
        self._assert_tesseract_state_equal(expected_state_1, state_1)
                        
        rospy.sleep(rospy.Duration(1))
        
        update_pose_proxy = rospy.ServiceProxy("update_payload_pose", UpdatePayloadPose)
        
        payload2_to_gripper_target_tf=self.tf_listener.lookupTransform("payload2", "payload2_gripper_target", rospy.Time(0))
               
        req = UpdatePayloadPoseRequest()
        req.header.frame_id = "vacuum_gripper_tool"
        req.name="payload2"
        req.pose = rox_msg.transform2pose_msg(payload2_to_gripper_target_tf.inv())
        req.confidence = 0.5
        res = update_pose_proxy(req)
        assert res.success
        
        expected_state_2 = rospy.wait_for_message("/expected_tesseract_diff_2", TesseractState, timeout=10)        
        state_2 = rospy.wait_for_message('/tesseract_diff', TesseractState, timeout=10)        
        self._assert_tesseract_state_equal(expected_state_2, state_2)
                      
        world_to_fixture2_payload2_target_tf=self.tf_listener.lookupTransform("world", "fixture2_payload2_target", rospy.Time(0))
        
        #Add in an offset to represent a final placement error
        fixture2_payload2_target_to_payload2_tf=rox.Transform(rox.rot([0,0,1], np.deg2rad(5)), [0.1,0,0], "fixture2_payload2_target", "payload2")
        
        req2 = UpdatePayloadPoseRequest()
        req2.header.frame_id = "world"
        req2.name="payload2"
        req2.pose = rox_msg.transform2pose_msg(world_to_fixture2_payload2_target_tf * fixture2_payload2_target_to_payload2_tf)
        res2 = update_pose_proxy(req2)
        assert res2.success
        
        expected_state_3 = rospy.wait_for_message("/expected_tesseract_diff_3", TesseractState, timeout=10)        
        state_3 = rospy.wait_for_message('/tesseract_diff', TesseractState, timeout=10)        
        self._assert_tesseract_state_equal(expected_state_3, state_3)
       
       
    
    def _assert_tesseract_state_equal(self, state1, state2):
        self.assertEqual(state1.name, state2.name)
        self.assertEqual(state1.urdf_name, state2.urdf_name)
        self.assertEqual(state1.object_colors, state2.object_colors)
        self.assertEqual(state1.highlight_links, state2.object_colors)
        self.assertEqual(state1.is_diff, state2.is_diff)
        
        self.assertEqual(len(state1.attachable_objects),len(state2.attachable_objects))
        for i in xrange(len(state1.attachable_objects)):
            self._assert_attachable_object_equal(
                state1.attachable_objects[i],
                state2.attachable_objects[i])
        
        self.assertEqual(len(state1.attached_bodies),len(state2.attached_bodies))
        for i in xrange(len(state1.attached_bodies)):
            self._assert_attached_body_equal(
                state1.attached_bodies[i],
                state2.attached_bodies[i])    
        
    
    def _assert_attachable_object_equal(self, obj1, obj2):
        self.assertEqual(obj1.name, obj2.name)
        self._assert_visual_geometry_equal(obj1.visual, obj2.visual)
        self._assert_collision_geometry_equal(obj1.collision, obj2.collision)
        
        self._assert_inertia_almost_equal(obj1.inertia,obj2.inertia)
             
        self.assertEqual(obj1.operation, obj2.operation)
    
    def _assert_visual_geometry_equal(self, vis1, vis2):        
        self.assertEqual(vis1.primitives, vis2.primitives)
        self._assert_pose_almost_equal(vis1.primitive_poses, vis2.primitive_poses)
        self.assertEqual(vis1.primitive_colors, vis2.primitive_colors)
        for m1, m2 in zip(vis1.meshes,vis2.meshes):
            self._assert_mesh_almost_equal(m1, m2)
        self._assert_pose_almost_equal(vis1.mesh_poses, vis2.mesh_poses)
        self.assertEqual(vis1.mesh_colors, vis2.mesh_colors)
        self.assertEqual(vis1.planes, vis2.planes)
        self._assert_pose_almost_equal(vis1.plane_poses, vis2.plane_poses)
        self.assertEqual(vis1.plane_colors, vis2.plane_colors)
        self.assertEqual(vis1.octomaps, vis2.octomaps)
        self._assert_pose_almost_equal(vis1.octomap_poses, vis2.octomap_poses)
        self.assertEqual(vis1.octomap_colors, vis2.octomap_colors)
    
    def _assert_collision_geometry_equal(self, col1, col2):
        self.assertEqual(col1.primitives, col2.primitives)
        self._assert_pose_almost_equal(col1.primitive_poses, col2.primitive_poses)
        self.assertEqual(col1.primitive_collision_object_types, col2.primitive_collision_object_types)
        self.assertEqual(col1.primitive_colors, col2.primitive_colors)
        for m1, m2 in zip(col1.meshes,col2.meshes):
            self._assert_mesh_almost_equal(m1, m2)
        self._assert_pose_almost_equal(col1.mesh_poses, col2.mesh_poses)
        self.assertEqual(col1.mesh_collision_object_types, col2.mesh_collision_object_types)
        self.assertEqual(col1.mesh_colors, col2.mesh_colors)
        self.assertEqual(col1.planes, col2.planes)
        self._assert_pose_almost_equal(col1.plane_poses, col2.plane_poses)
        self.assertEqual(col1.plane_collision_object_types, col2.plane_collision_object_types)
        self.assertEqual(col1.plane_colors, col2.plane_colors)
        self.assertEqual(col1.octomaps, col2.octomaps)
        self._assert_pose_almost_equal(col1.octomap_poses, col2.octomap_poses)
        self.assertEqual(col1.octomap_collision_object_types, col2.octomap_collision_object_types)
        self.assertEqual(col1.octomap_colors, col2.octomap_colors)
    
    def _assert_attached_body_equal(self, body1, body2):
        self.assertEqual(body1.object_name, body2.object_name)
        self.assertEqual(body1.parent_link_name, body2.parent_link_name)
        self._assert_pose_almost_equal(body1.transform, body2.transform)        
        self.assertEqual(body1.touch_links, body2.touch_links)        
        self.assertEqual(body1.operation, body2.operation)        
    
    
    def _assert_mesh_almost_equal(self, m1, m2):
        for t1, t2 in zip(m1.triangles, m2.triangles):
            np.testing.assert_equal(t1.vertex_indices,t2.vertex_indices)            
        for v1, v2 in zip(m1.vertices, m2.vertices):
            np.testing.assert_allclose([v1.x,v1.y,v1.z], [v2.x,v2.y,v2.z])
            
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
    def _assert_inertia_almost_equal(self, i1, i2):
        self.assertAlmostEqual(i1.m,i2.m)
        self.assertAlmostEqual(i1.com.x,i2.com.x)
        self.assertAlmostEqual(i1.com.y,i2.com.y)
        self.assertAlmostEqual(i1.com.z,i2.com.z)
        self.assertAlmostEqual(i1.ixx, i2.ixx)
        self.assertAlmostEqual(i1.ixy, i2.ixy)
        self.assertAlmostEqual(i1.ixz, i2.ixz)
        self.assertAlmostEqual(i1.ixy, i2.ixy)
        self.assertAlmostEqual(i1.iyy, i2.iyy)
        self.assertAlmostEqual(i1.iyz, i2.iyz)
        self.assertAlmostEqual(i1.izz, i2.izz)
    
    
if __name__ == "__main__":

    rospy.init_node("test_attached_collision_object", anonymous=True)
    
    import rostest
    rostest.rosrun("test_tesseract_diff", "TestTesseractDiff", TestTesseractDiff)
