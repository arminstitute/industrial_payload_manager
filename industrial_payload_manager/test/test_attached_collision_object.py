#!/usr/bin/env python

import rospy
import unittest
from industrial_payload_manager.msg import Payload, PayloadTarget, PayloadArray, \
    GripperTarget, ArucoGridboardWithPose, LinkMarkers
from industrial_payload_manager.srv import UpdatePayloadPose, UpdatePayloadPoseRequest
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
import numpy as np
import Queue
from industrial_payload_manager.payload_transform_listener import PayloadTransformListener
import general_robotics_toolbox as rox
import general_robotics_toolbox.ros_msg as rox_msg

class TestAttachedCollisionObject(unittest.TestCase):
    
    def __init__(self, *args, **kwargs):
        super(TestAttachedCollisionObject,self).__init__(*args, **kwargs)
        self._msg_queue = Queue.Queue()
        self.tf_listener=PayloadTransformListener()
    
    def _aco_cb(self, msg):
        self._msg_queue.put(msg)
    
    def test_attached_collision_object(self):
        payload_expected_msg = dict()
        payload_expected_msg["payload1"] = rospy.wait_for_message("/expected_attached_collision_object_1", AttachedCollisionObject, timeout=10)
        payload_expected_msg["payload2"] = rospy.wait_for_message("/expected_attached_collision_object_2", AttachedCollisionObject, timeout=10)
        payload_expected_msg["payload2_2"] = rospy.wait_for_message("/expected_attached_collision_object_2_2", AttachedCollisionObject, timeout=10)
        payload_expected_msg["payload2_3"] = rospy.wait_for_message("/expected_attached_collision_object_2_3", AttachedCollisionObject, timeout=10)
        payload_expected_msg["payload3"] = rospy.wait_for_message("/expected_attached_collision_object_3", AttachedCollisionObject, timeout=10)
        
        found_payload = {"payload1": False, "payload2": False, "payload3": False}
        found_remove_payload = {"payload1": False, "payload2": False, "payload3": False}
        
        rospy.Subscriber("/attached_collision_object", AttachedCollisionObject, self._aco_cb)
        
        while not all(found_payload.itervalues()):
            msg = self._msg_queue.get(timeout=10)
            if msg.object.operation == CollisionObject.REMOVE:
                assert msg.object.id in found_remove_payload
                found_remove_payload[msg.object.id] = True
            elif msg.object.operation == CollisionObject.ADD:
                assert msg.object.id in found_payload
                assert found_remove_payload[msg.object.id]
                found_payload[msg.object.id] = True
                expected_msg = payload_expected_msg[msg.object.id]
                self._assert_attached_collision_object_equal(msg, expected_msg)
                
         
        assert all(found_payload.itervalues())
        
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
        
        msg = self._msg_queue.get(timeout=10)
        assert msg.object.id == "payload2"
        assert msg.object.operation == CollisionObject.REMOVE
        
        msg = self._msg_queue.get(timeout=10)
        assert msg.object.id == "payload2"  
        assert msg.object.operation == CollisionObject.ADD
        self._assert_attached_collision_object_equal(msg, payload_expected_msg["payload2_2"])
        
        rospy.sleep(rospy.Duration(1))
        
        world_to_fixture2_payload2_target_tf=self.tf_listener.lookupTransform("world", "fixture2_payload2_target", rospy.Time(0))
        
        #Add in an offset to represent a final placement error
        fixture2_payload2_target_to_payload2_tf=rox.Transform(rox.rot([0,0,1], np.deg2rad(5)), [0.1,0,0], "fixture2_payload2_target", "payload2")
        
        req2 = UpdatePayloadPoseRequest()
        req2.header.frame_id = "world"
        req2.name="payload2"
        req2.pose = rox_msg.transform2pose_msg(world_to_fixture2_payload2_target_tf * fixture2_payload2_target_to_payload2_tf)
        res2 = update_pose_proxy(req2)
        assert res2.success
        
        msg = self._msg_queue.get(timeout=10)
        assert msg.object.id == "payload2"
        assert msg.object.operation == CollisionObject.REMOVE
        
        msg = self._msg_queue.get(timeout=10)
        assert msg.object.id == "payload2"  
        assert msg.object.operation == CollisionObject.ADD
        self._assert_attached_collision_object_equal(msg, payload_expected_msg["payload2_3"])
        
        
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
