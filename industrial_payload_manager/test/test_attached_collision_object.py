#!/usr/bin/env python

import rospy
import unittest
from industrial_payload_manager.msg import Payload, PayloadTarget, PayloadArray, \
    GripperTarget, ArucoGridboardWithPose, LinkMarkers
from industrial_payload_manager.srv import UpdatePayloadPose, UpdatePayloadPoseRequest
from moveit_msgs.msg import CollisionObject, PlanningScene
import numpy as np
import Queue
from industrial_payload_manager.payload_transform_listener import PayloadTransformListener
import general_robotics_toolbox as rox
import general_robotics_toolbox.ros_msg as rox_msg

class TestAttachedCollisionObject(unittest.TestCase):
    
    def __init__(self, *args, **kwargs):
        super(TestAttachedCollisionObject,self).__init__(*args, **kwargs)
        self.tf_listener=PayloadTransformListener()
            
    def test_attached_collision_object(self):
        
        expected_scene_1 = rospy.wait_for_message("/expected_planning_scene_1", PlanningScene, timeout=10)        
        scene_1 = rospy.wait_for_message('/planning_scene', PlanningScene, timeout=10)        
        self._assert_planning_scene_equal(expected_scene_1, scene_1)
                        
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
        
        expected_scene_2 = rospy.wait_for_message("/expected_planning_scene_2", PlanningScene, timeout=10)        
        scene_2 = rospy.wait_for_message('/planning_scene', PlanningScene, timeout=10)        
        self._assert_planning_scene_equal(expected_scene_2, scene_2)
                        
        world_to_fixture2_payload2_target_tf=self.tf_listener.lookupTransform("world", "fixture2_payload2_target", rospy.Time(0))
        
        #Add in an offset to represent a final placement error
        fixture2_payload2_target_to_payload2_tf=rox.Transform(rox.rot([0,0,1], np.deg2rad(5)), [0.1,0,0], "fixture2_payload2_target", "payload2")
        
        req2 = UpdatePayloadPoseRequest()
        req2.header.frame_id = "world"
        req2.name="payload2"
        req2.pose = rox_msg.transform2pose_msg(world_to_fixture2_payload2_target_tf * fixture2_payload2_target_to_payload2_tf)
        res2 = update_pose_proxy(req2)
        assert res2.success
        
        expected_scene_3 = rospy.wait_for_message("/expected_planning_scene_3", PlanningScene, timeout=10)        
        scene_3 = rospy.wait_for_message('/planning_scene', PlanningScene, timeout=10)        
        self._assert_planning_scene_equal(expected_scene_3, scene_3)
       
    
    def _assert_planning_scene_equal(self, scene1, scene2):
        self.assertEqual(scene1.name, scene2.name)
        self.assertEqual(scene1.robot_model_name, scene2.robot_model_name)
        self.assertEqual(scene1.fixed_frame_transforms, scene2.fixed_frame_transforms)
        self.assertEqual(scene1.link_padding, scene2.link_padding)
        self.assertEqual(scene1.link_scale, scene2.link_scale)
        self.assertEqual(scene1.object_colors, scene2.object_colors)
        self.assertEqual(scene1.world.collision_objects,scene2.world.collision_objects)
        self.assertEqual(scene1.is_diff, scene2.is_diff)
        
        self.assertEqual(scene1.robot_state.is_diff, scene2.robot_state.is_diff)
        self.assertEqual(len(scene1.robot_state.attached_collision_objects),len(scene2.robot_state.attached_collision_objects))
        for i in xrange(len(scene1.robot_state.attached_collision_objects)):
            self._assert_attached_collision_object_equal(
                scene1.robot_state.attached_collision_objects[i],
                scene2.robot_state.attached_collision_objects[i])
    
    def _assert_attached_collision_object_equal(self, aco1, aco2):
        self.assertEqual(aco1.link_name, aco2.link_name)
        self._assert_collision_object_equal(aco1.object, aco2.object)
        self.assertEqual(aco1.touch_links, aco2.touch_links)
        self.assertEqual(aco1.detach_posture, aco2.detach_posture)
        self.assertEqual(aco1.weight, aco2.weight)
        
    def _assert_collision_object_equal(self, co1, co2):
        self.assertEqual(co1.header.frame_id, co2.header.frame_id)
        self.assertEqual(co1.id, co2.id)
        self.assertEqual(co1.type, co2.type)
        self.assertEqual(co1.primitives, co2.primitives)
        self._assert_pose_almost_equal(co1.primitive_poses, co2.primitive_poses)
        for m1, m2 in zip(co1.meshes,co2.meshes):
            self._assert_mesh_almost_equal(m1, m2)
        self._assert_pose_almost_equal(co1.mesh_poses, co2.mesh_poses)
        self.assertEqual(co1.planes, co2.planes)
        self._assert_pose_almost_equal(co1.plane_poses, co2.plane_poses)
        self.assertEqual(co1.operation, co2.operation)
    
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
    
    
    
if __name__ == "__main__":

    rospy.init_node("test_attached_collision_object", anonymous=True)
    
    import rostest
    rostest.rosrun("test_attached_collision_object", "TestAttachedCollisionObject", TestAttachedCollisionObject)
