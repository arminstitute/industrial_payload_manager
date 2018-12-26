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

import general_robotics_toolbox as rox
from general_robotics_toolbox import ros_tf as rox_tf
from general_robotics_toolbox import ros_msg as rox_msg
from .msg import PayloadArray
from geometry_msgs.msg import TransformStamped, Vector3

import numpy as np
import rospy

from tf2_ros import TransformException, ConnectivityException, LookupException, ExtrapolationException

def _point_to_vector3(p):
    return Vector3(p.x, p.y, p.z)


class PayloadTransformListener(rox_tf.TransformListener):
    def __init__(self):
        super(PayloadTransformListener,self).__init__()
        
        self._payload_sub=rospy.Subscriber('payload', PayloadArray, self._payload_cb)
 
    def _payload_cb(self, msg):
                        
        for p in msg.payloads:
            t=TransformStamped()
            t.header.frame_id=p.header.frame_id
            t.header.stamp=p.header.stamp
            t.child_frame_id=p.name
            t.transform.rotation = p.pose.orientation
            t.transform.translation = _point_to_vector3(p.pose.position)
            self.ros_listener._buffer.set_transform_static(t,"default_authority")
            
            for g in p.gripper_targets:
                t=TransformStamped()
                t.header.frame_id=p.name
                t.header.stamp=p.header.stamp
                t.child_frame_id=g.name
                t.transform.rotation = g.pose.orientation
                t.transform.translation = _point_to_vector3(g.pose.position)
                self.ros_listener._buffer.set_transform_static(t,"default_authority")
                
            for m in p.markers:
                t=TransformStamped()
                t.header.frame_id=p.name
                t.header.stamp=p.header.stamp
                t.child_frame_id=m.name
                t.transform.rotation = m.pose.orientation
                t.transform.translation = _point_to_vector3(m.pose.position)
                self.ros_listener._buffer.set_transform_static(t,"default_authority")
            
        for p_t in msg.payload_targets:
            t=TransformStamped()
            t.header.frame_id=p_t.header.frame_id
            t.header.stamp=p_t.header.stamp
            t.child_frame_id=p_t.name
            t.transform.rotation = p_t.pose.orientation
            t.transform.translation = _point_to_vector3(p_t.pose.position)
            self.ros_listener._buffer.set_transform_static(t,"default_authority") 
        
        for p_m in msg.link_markers:
            for m in p_m.markers:
                t=TransformStamped()
                t.header.frame_id=p_m.header.frame_id
                t.header.stamp=p_m.header.stamp
                t.child_frame_id=m.name
                t.transform.rotation = m.pose.orientation
                t.transform.translation = _point_to_vector3(m.pose.position)
                self.ros_listener._buffer.set_transform_static(t,"default_authority")   
        
