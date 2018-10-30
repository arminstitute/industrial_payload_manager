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
from .msg import Payload, PayloadTarget, PayloadArray, \
    GripperTarget, ArucoGridboardWithPose, LinkMarkers
from .srv import UpdatePayloadPose, UpdatePayloadPoseRequest, UpdatePayloadPoseResponse, \
    GetPayloadArray, GetPayloadArrayRequest, GetPayloadArrayResponse
from moveit_commander import PlanningSceneInterface
import general_robotics_toolbox.ros_msg as rox_msg
from visualization_msgs.msg import Marker, MarkerArray
from urdf_parser_py.urdf import URDF
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject

class Payload(object):
    def __init__(self, payload_msg, ros_id):
        self.payload_msg=payload_msg
        self.ros_id=ros_id
        self.attached_link=None

class PayloadManagerPlanningSceneInterface(PlanningSceneInterface):
    def __init__(self, ns='', pub_aco=None):
        super(PayloadManagerPlanningSceneInterface, self).__init__(ns)
        if pub_aco is not None:
            self._pub_aco=pub_aco

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
        self._pub_aco_listener = PayloadManagerSubscriberListener(self)
        self._pub_aco=rospy.Publisher('/attached_collision_object', AttachedCollisionObject, queue_size=100, subscriber_listener = self._pub_aco_listener)
        self.planning_scene=PayloadManagerPlanningSceneInterface(pub_aco = self._pub_aco)
        self._payload_msg_pub=rospy.Publisher("payload", PayloadArray, queue_size=100)
        self.rviz_cam_publisher=rospy.Publisher("rviz_sim_cameras/payload_marker_array", MarkerArray, queue_size=100, latch=True)
        self._payload_msg_sub=rospy.Subscriber("payload", PayloadArray, self._payload_msg_cb)
        self._update_payload_pose_srv=rospy.Service("update_payload_pose", UpdatePayloadPose, self._update_payload_pose_srv_cb)
        self._get_payload_array_srv=rospy.Service("get_payload_array", GetPayloadArray, self._get_payload_array_srv_cb)

    def _payload_msg_cb(self, msg):
        try:
            with self.payloads_lock:            
                for p in msg.payloads:
                    if p.name in self.payload_targets or p.name in self.link_markers:
                        rospy.logerr("Payload name already in use %s", p.name)
                        continue
                    if p.name not in self.payloads:
                        ros_id=self._ros_id
                        self._ros_id += 1
                        payload=Payload(p, ros_id)
                        self.payloads[p.name]=payload
                        self._update_payload_mesh(payload)
                    else:
                        payload = self.payloads[p.name]
                        #ignore stale data
                        if payload.payload_msg.header.stamp > p.header.stamp:
                            continue
                        
                        payload.payload_msg=p                
                        self._update_payload_mesh(payload) 
                    
                for t in msg.payload_targets:
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
                
                for l in msg.link_markers:
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
                
                for d in msg.delete_payloads:
                    if d in self.payloads:
                        payload = self.payloads[d]
                        del self.payloads[d]
                        self._delete_payload_mesh(payload)                        
                    if d in self.payload_targets:
                        del self.payload_targets[d]
                    if d in self.link_markers:
                        del self.link_markers[d]
        except:
            traceback.print_exc()
            
    def _update_payload_mesh(self, payload):
        rospy.logdebug("Update payload mesh %s", payload.payload_msg.name)
        
        self._remove_payload_from_planning_scene(payload)
        self._add_payload_to_planning_scene(payload)
        self._update_rviz_sim_cameras()
        
            
    
    def _delete_payload_mesh(self, payload):
        rospy.logdebug("Delete payload mesh %s", payload.payload_msg.name)
        self._update_rviz_sim_cameras()
        self._remove_payload_from_planning_scene(payload)
    
    def _remove_payload_from_planning_scene(self, payload):
        msg=payload.payload_msg
                        
        if payload.attached_link is not None:
            if len(msg.mesh_resources) > 0:
                try:
                    self.planning_scene.remove_attached_object(payload.attached_link, msg.name)
                except:
                    pass
                
                for i in xrange(1,len(msg.mesh_resources)):
                    try:
                        self.planning_scene.remove_attached_object(payload.attached_link, msg.name + "_%d" % i)
                    except:
                        pass
        
        payload.attached_link = None
            
    def _add_payload_to_planning_scene(self, payload):
        
        msg=payload.payload_msg
        
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
                
        for i in xrange(len(msg.mesh_resources)):
            
            mesh_name=msg.name
            if i > 0: mesh_name += "_%d" % i
            
            mesh_filename=urlparse.urlparse(resource_retriever.get_filename(msg.mesh_resources[i])).path
            if mesh_filename.endswith(".dae"):
                
                #TODO: fix the dae import in planning_scene_interface
                rospy.logwarn("dae files currently don't work with python planning_scene_interface, ignoring %s", mesh_filename)
                continue
            
            mesh_pose = rox_msg.transform2pose_stamped_msg(rox_msg.msg2transform(msg.pose) * rox_msg.msg2transform(msg.mesh_poses[i]))            
            mesh_pose.header.frame_id = payload.attached_link            
            self.planning_scene.attach_mesh(payload.attached_link, mesh_name, mesh_pose, mesh_filename, touch_links=touch_links)
    
    def _update_rviz_sim_cameras(self):
        
        marker_array=MarkerArray()
        for p in self.payloads:
            payload=self.payloads[p]
            msg=payload.payload_msg           
            
            for i in xrange(len(msg.mesh_resources)):
                marker=Marker()
                marker.ns="payload_" + msg.name
                marker.id=i
                marker.type=marker.MESH_RESOURCE
                marker.action=marker.ADD     
                marker.mesh_resource=msg.mesh_resources[i]
                marker.mesh_use_embedded_materials=True
                
                marker.pose = rox_msg.transform2pose_msg(rox_msg.msg2transform(msg.pose) * rox_msg.msg2transform(msg.mesh_poses[i]))                     
                marker.header.frame_id = payload.attached_link
                marker.scale.x=1.0
                marker.scale.y=1.0
                marker.scale.z=1.0
                marker.color.a=1
                marker.color.r=0.5
                marker.color.g=0.5
                marker.color.b=0.5            
                marker.header.stamp=rospy.Time.now()
                marker.frame_locked=True
                marker._check_types()
                #marker.colors.append(ColorRGBA(0.5,0.5,0.5,1))
            
                marker_array.markers.append(marker)               
                    
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
            
            

def industrial_payload_manager_main():
    
    rospy.init_node("industrial_payload_manager")
    
    manager = PayloadManager()
    rospy.spin()
    