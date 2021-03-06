# Copyright (c) 2018, Rensselaer Polytechnic Institute, Wason Technology LLC
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Rensselaer Polytechnic Institute, nor Wason 
#       Technology LLC, nor the names of its contributors may be used to 
#       endorse or promote products derived from this software without 
#       specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import absolute_import

import rospy
import threading
import traceback
import urlparse
import resource_retriever
import copy
import numpy as np
from .msg import Payload, PayloadTarget, PayloadArray, \
    GripperTarget, ArucoGridboardWithPose, LinkMarkers
from .srv import UpdatePayloadPose, UpdatePayloadPoseRequest, UpdatePayloadPoseResponse, \
    GetPayloadArray, GetPayloadArrayRequest, GetPayloadArrayResponse
import general_robotics_toolbox.ros_msg as rox_msg
from visualization_msgs.msg import Marker, MarkerArray
from urdf_parser_py.urdf import URDF
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from geometry_msgs.msg import Point, Pose, Vector3
from std_msgs.msg import Int32

# Load pyassimp, based on moveit_commander planning_scene_interface.py
try:
    from pyassimp import pyassimp
except:
    # support pyassimp > 3.0
    try:
        import pyassimp
    except:
        pyassimp = False
        print("Failed to import pyassimp, see https://github.com/ros-planning/moveit/issues/86 for more info")

try:
    from tesseract_msgs.msg import  TesseractState, AttachableObject, AttachedBodyInfo
    from tesseract_msgs.srv import ModifyTesseractEnv, ModifyTesseractEnvRequest
    _use_tesseract=True
except:
    _use_tesseract=False

class Payload(object):
    def __init__(self, payload_msg, ros_id):
        self.payload_msg=payload_msg
        self.ros_id=ros_id
        self.attached_link=None

class PayloadManagerSubscriberListener(rospy.SubscribeListener):
    def __init__(self, manager):
        self._manager = manager        
    
    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        self._manager.publish_moveit_aco()
    
class PayloadManager(object):
    def __init__(self):
        self.urdf=URDF.from_parameter_server()
        self.payloads=dict()
        self.payload_targets=dict()
        self.link_markers=dict()
        self.payloads_lock=threading.Lock()
        self._ros_id=1
        #self._pub_aco_listener = PayloadManagerSubscriberListener(self)
        #self._pub_aco=rospy.Publisher('/attached_collision_object', AttachedCollisionObject, queue_size=100, subscriber_listener = self._pub_aco_listener)
        self._pub_planning_scene=rospy.Publisher("planning_scene", PlanningScene, latch = True, queue_size=10)        
        self._payload_msg_pub=rospy.Publisher("payload", PayloadArray, queue_size=100)
        self.rviz_cam_publisher=rospy.Publisher("payload_marker_array", MarkerArray, queue_size=100, latch=True)
        self._payload_msg_sub=rospy.Subscriber("payload", PayloadArray, self._payload_msg_cb)
        self._update_payload_pose_srv=rospy.Service("update_payload_pose", UpdatePayloadPose, self._update_payload_pose_srv_cb)
        self._get_payload_array_srv=rospy.Service("get_payload_array", GetPayloadArray, self._get_payload_array_srv_cb)

        if _use_tesseract:
            self._tesseract_pub = rospy.Publisher("tesseract_diff", TesseractState, latch=True, queue_size=10)
            

    def _payload_msg_cb(self, msg):
        with self.payloads_lock:
            try:
                for p in msg.payloads:
                    try:
                        if p.name in self.payload_targets or p.name in self.link_markers:
                            rospy.logerr("Payload name already in use %s", p.name)
                            continue
                        if p.name not in self.payloads:
                            ros_id=self._ros_id
                            self._ros_id += 1
                            payload=Payload(p, ros_id)
                            self.payloads[p.name]=payload
                            #self._update_payload_mesh(payload)
                        else:
                            payload = self.payloads[p.name]
                            #ignore stale data
                            if payload.payload_msg.header.stamp > p.header.stamp:
                                continue
                            
                            payload.payload_msg=p                
                            #self._update_payload_mesh(payload)
                        rospy.loginfo("Updated payload " + p.name )
                    except:
                        traceback.print_exc()
                        rospy.logger("Error processing payload " + p.name)
                    
                for t in msg.payload_targets:
                    try:
                        if t.name in self.payloads or t.name in self.link_markers:
                            rospy.logerr("Payload name already in use %s", t.name)
                            continue
                        if t.name not in self.payload_targets:                    
                            self.payload_targets[t.name]=t                    
                        else:
                            payload_target = self.payload_targets[t.name]
                            #ignore stale data
                            if payload_target.header.stamp > t.header.stamp:
                                continue
                            self.payload_targets[t.name]=t
                        rospy.loginfo("Updated payload target " + t.name)
                    except:
                        traceback.print_exc()
                        rospy.logger("Error processing payload target " + t.name)
                
                for l in msg.link_markers:
                    try:
                        if l.header.frame_id in self.payloads or l.header.frame_id in self.payload_targets:
                            rospy.logerr("Payload name already in use %s", t.name)
                            continue
                        if l.header.frame_id not in self.link_markers:                    
                            self.link_markers[l.header.frame_id]=l                    
                        else:
                            link_marker = self.link_markers[l.header.frame_id]
                            #ignore stale data
                            if link_marker.header.stamp > l.header.stamp:
                                continue
                            self.link_markers[l.header.frame_id]=l
                        rospy.loginfo("Updated link marker " + l.header.frame_id )
                    except:
                        traceback.print_exc()
                        rospy.logger("Error processing link_marker " + l.header.frame_id)
                
                delete_payloads=[]
                try:
                    for d in msg.delete_payloads:
                        if d in self.payloads:
                            payload = self.payloads[d]
                            del self.payloads[d]
                            delete_payloads.append(payload)
                            rospy.loginfo("Deleted payload " + d )
                            #self._delete_payload_mesh(payload)                        
                        if d in self.payload_targets:
                            del self.payload_targets[d]
                            rospy.loginfo("Deleted payload target " + d )
                        if d in self.link_markers:
                            del self.link_markers[d]
                            rospy.loginfo("Deleted link marker " + d )
                except:
                    traceback.print_exc()
                    rospy.logger("Error deleting payload " + d.name)
            
                try:
                    self._publish_planning_scene(delete_payloads)
                except:
                    traceback.print_exc()
                    rospy.logger("Error publishing planning scene")
                try:
                    self._publish_rviz_sim_cameras()
                except:
                    traceback.print_exc()
                    rospy.logger("Error publishing rviz markers")
                if (_use_tesseract):
                    try:
                        self._publish_tesseract_scene(delete_payloads)
                    except:
                        traceback.print_exc()
                        rospy.logger("Error publishing tesseract scene")
            except:
                traceback.print_exc()
    
    def _publish_planning_scene(self, delete_payloads):
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.robot_state.is_diff=True
        
        for p in self.payloads.itervalues():
            aco_remove = self._remove_payload_from_planning_scene_msg(p)
            aco_add = self._add_payload_to_planning_scene_msg(p)
            planning_scene.robot_state.attached_collision_objects.append(aco_remove)
            planning_scene.robot_state.attached_collision_objects.append(aco_add)
        
        for d in delete_payloads:
            aco = self._remove_payload_from_planning_scene_msg(d)
            planning_scene.robot_state.attached_collision_objects.append(aco)
            
        self._pub_planning_scene.publish(planning_scene)
        
    def _publish_tesseract_scene(self, delete_payloads):
        env=TesseractState()
        env.is_diff=True
        for p in self.payloads.itervalues():
            body_remove = self._remove_payload_from_tesseract_msg(p)
            attach_obj, body_add = self._add_payload_to_tesseract_msgs(p)
            env.attached_bodies.append(body_remove)
            env.attached_bodies.append(body_add)
            env.attachable_objects.append(attach_obj)
        
        for d in delete_payloads:
            body_remove = self._remove_payload_from_tesseract_msg(d)
            env.attached_bodies.append(body_remove)
    
        self._tesseract_pub.publish(env)
    
    #def _update_payload_mesh(self, payload):
    #    rospy.logdebug("Update payload mesh %s", payload.payload_msg.name)
    #   
    #    self._remove_payload_from_planning_scene(payload)
    #    self._add_payload_to_planning_scene(payload)
    #    self._update_rviz_sim_cameras()
        
            
    
    #def _delete_payload_mesh(self, payload):
    #    rospy.logdebug("Delete payload mesh %s", payload.payload_msg.name)
    #    self._update_rviz_sim_cameras()
    #    self._remove_payload_from_planning_scene(payload)
    
    def _remove_payload_from_planning_scene_msg(self, payload):
        msg=payload.payload_msg
        
        aco = AttachedCollisionObject()
        aco.object.operation = CollisionObject.REMOVE
        if payload.attached_link is not None:
            aco.link_name = payload.attached_link
        else:
            aco.link_name = ""
        aco.object.id = msg.name
        #self._pub_aco.publish(aco)
        
        payload.attached_link = None
        return aco
            
    def _add_payload_to_planning_scene_msg(self, payload):
        
        msg=payload.payload_msg
        
        co = CollisionObject()
        co.id = payload.payload_msg.name
        co.header.frame_id = msg.header.frame_id
        
        payload.attached_link=msg.header.frame_id
        
        urdf_root=self.urdf.get_root()
        urdf_chain=self.urdf.get_chain(urdf_root, payload.attached_link, joints=True, links=False)
        urdf_chain.reverse()
        touch_links=[]
        touch_root = None        
        for j_name in urdf_chain:
            j=self.urdf.joint_map[j_name]
            if j.type != "fixed":
                break
            touch_root=j.parent
        
        def _touch_recurse(touch):
            ret=[touch]
            if touch in self.urdf.child_map:                
                for c_j,c in self.urdf.child_map[touch]:
                    if self.urdf.joint_map[c_j].type == "fixed":
                        ret.extend(_touch_recurse(c))
            return ret
        
        if touch_root is not None:
            touch_links.extend(_touch_recurse(touch_root))        
        
        for i in xrange(len(msg.collision_geometry.mesh_resources)):
            
            mesh_filename=urlparse.urlparse(resource_retriever.get_filename(msg.collision_geometry.mesh_resources[i])).path
            mesh_pose = rox_msg.transform2pose_msg(rox_msg.msg2transform(msg.pose) * rox_msg.msg2transform(msg.collision_geometry.mesh_poses[i]))            
            mesh_scale=msg.collision_geometry.mesh_scales[i]
            mesh_scale = (mesh_scale.x, mesh_scale.y, mesh_scale.z)
            
            mesh_msg = load_mesh_file_to_mesh_msg(mesh_filename, mesh_scale)
            co.meshes.extend(mesh_msg)
            co.mesh_poses.extend([mesh_pose]*len(mesh_msg))
        
        for i in xrange(len(msg.collision_geometry.primitives)):
            co.primitives.append(msg.collision_geometry.primitives[i])
            primitive_pose = rox_msg.transform2pose_msg(rox_msg.msg2transform(msg.pose) * rox_msg.msg2transform(msg.collision_geometry.primitive_poses[i]))
            co.primitive_poses.append(primitive_pose)
        
        aco = AttachedCollisionObject()    
        aco.link_name = payload.attached_link
        aco.object = co
        aco.touch_links = touch_links
        aco.weight = msg.inertia.m 
        
        #self._pub_aco.publish(aco)
        return aco
    
    def _remove_payload_from_tesseract_msg(self, payload):
        msg=payload.payload_msg
        
        body_info = AttachedBodyInfo()
        body_info.operation = AttachedBodyInfo.REMOVE
        
        body_info.object_name = msg.name
        #self._pub_aco.publish(aco)        
        payload.attached_link = None
        return body_info
    
    def _add_payload_to_tesseract_msgs(self, payload):
        
        if not _use_tesseract:
            return
        msg=payload.payload_msg
        
        payload.attached_link=msg.header.frame_id
                
        attach_obj = AttachableObject()
        attach_obj.name = payload.payload_msg.name
                
        urdf_root=self.urdf.get_root()
        urdf_chain=self.urdf.get_chain(urdf_root, payload.attached_link, joints=True, links=False)
        urdf_chain.reverse()
        touch_links=[]
        touch_root = None        
        for j_name in urdf_chain:
            j=self.urdf.joint_map[j_name]
            if j.type != "fixed":
                break
            touch_root=j.parent
        
        def _touch_recurse(touch):
            ret=[touch]
            if touch in self.urdf.child_map:                
                for c_j,c in self.urdf.child_map[touch]:
                    if self.urdf.joint_map[c_j].type == "fixed":
                        ret.extend(_touch_recurse(c))
            return ret
        
        if touch_root is not None:
            touch_links.extend(_touch_recurse(touch_root))        
        
        for i in xrange(len(msg.collision_geometry.mesh_resources)):
            
            mesh_filename=urlparse.urlparse(resource_retriever.get_filename(msg.collision_geometry.mesh_resources[i])).path
            mesh_pose = msg.collision_geometry.mesh_poses[i]            
            mesh_scale=msg.collision_geometry.mesh_scales[i]
            mesh_scale = (mesh_scale.x, mesh_scale.y, mesh_scale.z)
                        
            mesh_msg = load_mesh_file_to_mesh_msg(mesh_filename, mesh_scale)
            attach_obj.collision.meshes.extend(mesh_msg)
            attach_obj.collision.mesh_poses.extend([mesh_pose]*len(mesh_msg))
            attach_obj.collision.mesh_collision_object_types.extend([Int32(0)]*len(mesh_msg))
            attach_obj.visual.meshes.extend(mesh_msg)
            attach_obj.visual.mesh_poses.extend([mesh_pose]*len(mesh_msg))
                                    
            if len(msg.collision_geometry.mesh_colors) > i:
                mesh_color=msg.collision_geometry.mesh_colors[i]
                attach_obj.visual.mesh_colors.extend([mesh_color]*len(mesh_msg))
                attach_obj.collision.mesh_colors.extend([mesh_color]*len(mesh_msg))
        
        for i in xrange(len(msg.collision_geometry.primitives)):
            attach_obj.collision.primitives.append(msg.collision_geometry.primitives[i])
            primitive_pose = msg.collision_geometry.primitive_poses[i]
            attach_obj.collision.primitive_poses.append(primitive_pose)
            attach_obj.collision.primitive_collision_object_types.append(Int32(0))
            attach_obj.visual.primitives.append(msg.collision_geometry.primitives[i])
            attach_obj.visual.primitive_poses.append(primitive_pose)            
            if len(msg.collision_geometry.mesh_colors) > i:
                attach_obj.collision.primitive_colors.append(msg.collision_geometry.mesh_colors[i])
                attach_obj.visual.primitive_colors.append(msg.collision_geometry.mesh_colors[i])
        attach_obj.inertia = msg.inertia
        
        body_info = AttachedBodyInfo()        
        body_info.parent_link_name=msg.header.frame_id
        body_info.object_name=payload.payload_msg.name
        body_info.transform=msg.pose
        body_info.touch_links=touch_links
                
        #self._pub_aco.publish(aco)
        return attach_obj, body_info
    
    def _publish_rviz_sim_cameras(self):
        
        marker_array=MarkerArray()
        for p in self.payloads:
            payload=self.payloads[p]
            msg=payload.payload_msg           
            
            id = 0
            
            for i in xrange(len(msg.visual_geometry.mesh_resources)):
                marker=Marker()
                marker.ns="payload_" + msg.name
                marker.id=id
                marker.type=marker.MESH_RESOURCE
                marker.action=marker.ADD     
                marker.mesh_resource=msg.visual_geometry.mesh_resources[i]
                marker.mesh_use_embedded_materials=True
                
                marker.pose = rox_msg.transform2pose_msg(rox_msg.msg2transform(msg.pose) * rox_msg.msg2transform(msg.visual_geometry.mesh_poses[i]))                     
                marker.header.frame_id = payload.attached_link
                marker.scale=msg.visual_geometry.mesh_scales[i]
                if i < len(msg.visual_geometry.mesh_colors):
                    marker.color=msg.visual_geometry.mesh_colors[i]
                else:                
                    marker.color.a=1
                    marker.color.r=0.5
                    marker.color.g=0.5
                    marker.color.b=0.5            
                marker.header.stamp=rospy.Time.now()
                marker.frame_locked=True
                marker._check_types()
                #marker.colors.append(ColorRGBA(0.5,0.5,0.5,1))
            
                marker_array.markers.append(marker)
                id += 1 
            for i in xrange(len(msg.visual_geometry.primitives)):
                marker=Marker()
                marker.ns="payload_" + msg.name
                marker.id=id
                primitive = msg.visual_geometry.primitives[i]
                primitive_type = primitive.type
                if primitive_type == SolidPrimitive.BOX:
                    marker.type=Marker.CUBE
                    assert len(primitive.dimensions) == 3
                    marker.scale = Vector3(*primitive.dimensions)
                elif primitive_type == SolidPrimitive.CYLINDER:
                    marker.type=Marker.CYLINDER
                    assert len(primitive.dimensions) == 2
                    marker.scale = Vector3(primitive.dimensions[1]*2, primitive.dimensions[1]*2, primitive.dimensions[0])
                elif primitive_type == SolidPrimitive.SPHERE:
                    marker.type=Marker.SPHERE
                    assert len(primitive.dimensions) == 1
                    marker.scale = Vector3(*([primitive.dimensions[0]*2]*3))
                    
                else:
                    assert False, "Invalid geometry specified for SolidPrimitive"
                
                marker.action=marker.ADD                             
                
                marker.pose = rox_msg.transform2pose_msg(rox_msg.msg2transform(msg.pose) * rox_msg.msg2transform(msg.visual_geometry.primitive_poses[i]))                     
                marker.header.frame_id = payload.attached_link                
                if i < len(msg.visual_geometry.primitive_colors):
                    marker.color=msg.visual_geometry.primitive_colors[i]
                else:                
                    marker.color.a=1
                    marker.color.r=0.5
                    marker.color.g=0.5
                    marker.color.b=0.5            
                marker.header.stamp=rospy.Time.now()
                marker.frame_locked=True
                marker._check_types()
                marker_array.markers.append(marker) 
                id += 1
                
        marker_array._check_types
        self.rviz_cam_publisher.publish(marker_array)            
    
    def _update_payload_pose(self, payload_name, pose, parent_frame_id = None, confidence = 0.1):
        with self.payloads_lock:
            n=rospy.Time.now()
            payload=copy.deepcopy(self.payloads[payload_name].payload_msg)
            if parent_frame_id is None:
                parent_frame_id = payload.header.frame_id
                
            parent_tf = self.tf_listener.lookupTransform(parent_frame_id, pose.parent_frame_id, rospy.Time(0))
            pose2=parent_tf.inv() * pose
            payload.pose=rox_msg.transform2pose_msg(pose2)
            payload.header.frame_id=parent_frame_id            
            payload.header.stamp=n
            payload.confidence = confidence
            
            payload_a=PayloadArray()
            payload_a.payloads.append(payload)
            payload_a.header.stamp=n
            self._payload_msg_pub.publish(payload_a)        
    
    def _update_payload_pose_srv_cb(self, req):
        with self.payloads_lock:
            try:
                n=rospy.Time.now()
                payload=copy.deepcopy(self.payloads[req.name].payload_msg)
                                
                payload.pose=req.pose
                payload.header.frame_id=req.header.frame_id       
                payload.header.stamp=n
                payload.confidence = req.confidence
                
                payload_a=PayloadArray()
                payload_a.payloads.append(payload)
                payload_a.header.stamp=n
                self._payload_msg_pub.publish(payload_a)  
            except:
                traceback.print_exc()
                rospy.logerr("Could not update pose for payload " + req.name)
                return UpdatePayloadPoseResponse(False)
            return UpdatePayloadPoseResponse(True)
    
    def publish_moveit_aco(self):
        try:
            with self.payloads_lock:
                for p in self.payloads.itervalues():
                    self._update_payload_mesh(p)                    
        except:
            traceback.print_exc()
            
    def _get_payload_array_srv_cb(self, req):
        p_array = PayloadArray()
        with self.payloads_lock:      
            try:
                for p, v in self.payloads.iteritems():
                    if not req.payload_names or p in req.payload_names: 
                        p_array.payloads.append(v.payload_msg)
                for p, v in self.payload_targets.iteritems():
                    if not req.payload_names or p in req.payload_names:
                        p_array.payload_targets.append(v)
                for p, v in self.link_markers.iteritems():
                    if not req.payload_names or p in req.payload_names:
                        p_array.link_markers.append(v)           
            except:
                traceback.print_exc()
                return GetPayloadArrayResponse(False, PayloadArray())
            return GetPayloadArrayResponse(True, p_array)
            
def _get_meshes_from_pyassimp_node(node, scale):
    meshes=[]
    tf = node.transformation
    # Match behavior of RViz for loading collada data
    # See rviz mesh_loader.cpp buildMesh() lines 227-236
    pnode = node.parent
    while pnode is not None and hasattr(pnode, 'parent'):
        # Don't convert to y-up orientation, which is what the root node in
        # Assimp does
        if pnode.parent is not None and hasattr(pnode.parent, 'parent'):
            tf = np.dot(pnode.transformation, tf)
        pnode = pnode.parent
    for m in node.meshes:
        mesh = Mesh()
        for face in m.faces:
            triangle = MeshTriangle()
            triangle.vertex_indices = [face[0], face[1], face[2]]
            mesh.triangles.append(triangle)
        for vertex in m.vertices:
            point = Point()
            vertex_h=np.dot(tf,np.hstack((vertex, [1])))
            point.x = vertex_h[0] * scale[0]
            point.y = vertex_h[1] * scale[1]
            point.z = vertex_h[2] * scale[2]
            mesh.vertices.append(point)
        meshes.append(mesh)
        
    for c in node.children:
        meshes.extend( _get_meshes_from_pyassimp_node(c,scale))        
    return meshes

def load_mesh_file_to_mesh_msg(filename, scale = (1,1,1)):
    # Load mesh from file
    # Loosely based on planning_scene_interface in moveit_commander package
    if pyassimp is False:
        raise Exception("Pyassimp needs patch https://launchpadlibrarian.net/319496602/patchPyassim.txt")
    
    scene = pyassimp.load(filename)
    return _get_meshes_from_pyassimp_node(scene.rootnode, scale)

def industrial_payload_manager_main():
    
    rospy.init_node("industrial_payload_manager")
    
    manager = PayloadManager()
    rospy.spin()
    