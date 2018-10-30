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

import xacro
from urdf_parser_py.urdf import URDF, Mesh
from .msg import Payload, PayloadTarget, PayloadArray, \
    GripperTarget, ArucoGridboardWithPose, LinkMarkers
import argparse
import general_robotics_toolbox as rox
import general_robotics_toolbox.ros_msg as rox_msg
import re
import numpy as np
import xml.etree.ElementTree as ET
import rospy
from itertools import chain

def _origin_to_pose(origin):
    R = rox.rpy2R(origin.rpy) if origin.rpy is not None else np.eye(3)
    p = origin.xyz if origin.xyz is not None else np.zeros((3,))
    t=rox.Transform(R, p)
    return rox_msg.transform2pose_msg(t)

def xacro_string_to_payload(xacro_str, xacro_fname=None, **kwargs):
    
    if xacro_fname is not None:
        xacro.restore_filestack([xacro_fname])
    else:
        xacro.restore_filestack([])
    
    if 'mappings' not in kwargs:
        kwargs['mappings']={'payload_extensions': 'true'}
    
    doc = xacro.parse(xacro_str)       
    xacro.process_doc(doc, **kwargs)   
    urdf_str = doc.toprettyxml(indent='  ')
    
    return urdf_to_payload(urdf_str)
    
    

def urdf_to_payload(xml_str):
    urdf_robot = URDF.from_xml_string(xml_str)
    urdf_et = ET.fromstring(xml_str)
    
    
    base_links = [u for u in urdf_robot.child_map.keys() if u not in urdf_robot.parent_map]

    assert len(base_links) == 1, "Multiple payloads detected, invalid URDF."
    
    base_link=base_links[0]
    
    assert base_link not in urdf_robot.link_map, "Invalid initial_pose parent link in payload URDF"
    assert len(urdf_robot.child_map[base_link]) == 1, "Invalid initial_pose in payload URDF"
    (initial_pose_joint_name, payload_link_name)=urdf_robot.child_map[base_link][0]
    
    assert initial_pose_joint_name.endswith('_initial_pose'), "Invalid initial_pose parent link in payload URDF"
    #assert all([j.type == "fixed" for _, j in urdf_robot.joint_map.items()]), "All joints must be fixed type in payload URDF"
    assert all([urdf_robot.parent_map[l][1] == payload_link_name for l in urdf_robot.link_map if l != payload_link_name]), \
        "All links must have payload link as parent"
    
    payload_link=urdf_robot.link_map[payload_link_name]
    initial_pose_joint=urdf_robot.joint_map[initial_pose_joint_name]
    
    payload_msg = Payload()        
    payload_msg.name = payload_link_name
    
    #Load in initial pose
    payload_msg.header.frame_id=initial_pose_joint.parent    
    payload_msg.pose=_origin_to_pose(initial_pose_joint.origin)
    
    #Load in gripper targets    
    for _, l in urdf_robot.link_map.items():
        m = re.match(r'^\w+_gripper_target(?:_(\d+))?$',l.name)
        if m is None:
            continue                 
                
        j = urdf_robot.joint_map[urdf_robot.parent_map[l.name][0]]
        
        assert j.parent == payload_link_name, "Invalid gripper_target for payload in URDF"
        
        pose=_origin_to_pose(j.origin)
        
        gripper_target=GripperTarget()
        gripper_target.header.frame_id = payload_link_name
        gripper_target.name=l.name
        gripper_target.pose=pose
        
        link_et=urdf_et.find('.//link[@name="' + l.name + '"]')
        ft_et=link_et.find('.//gripper_ft_threshold')
        if ft_et is not None:
            pickup_et=ft_et.find('.//pickup')
            if pickup_et is not None:
                ft_val=[float(d) for d in pickup_et.attrib['ft'].split()]
                assert len(ft_val) == 6, "Invalid pickup ft"
                gripper_target.pickup_ft_threshold=ft_val
            
            place_et=ft_et.find('.//place')
            if place_et is not None:
                ft_val=[float(d) for d in place_et.attrib['ft'].split()]
                assert len(ft_val) == 6, "Invalid pickup ft"
                gripper_target.place_ft_threshold=ft_val
        
        payload_msg.gripper_targets.append(gripper_target)  
    
    #Load in markers    
    for _, l in urdf_robot.link_map.items():
        m = re.match(r'^\w+_marker(?:_(\d+))?$',l.name)
        if m is None:
            continue                 
                
        j = urdf_robot.joint_map[urdf_robot.parent_map[l.name][0]]
        
        assert j.parent == payload_link_name, "Invalid marker for payload in URDF"
        
        pose=_origin_to_pose(j.origin)
        aruco_marker = next((x for x in payload_msg.markers if x.name == l), None)
        
        aruco_marker=ArucoGridboardWithPose()
        aruco_marker.header.frame_id = payload_link_name
        aruco_marker.name=l.name
        aruco_marker.pose=pose
        
        link_et=urdf_et.find('.//link[@name="' + l.name + '"]')
        marker_et=link_et.find('.//aruco_marker')
        if marker_et is not None:
            gridboard_et=marker_et.find('.//gridboard')
            if gridboard_et is not None:
                a=gridboard_et.attrib
                aruco_marker.marker.markersX=int(a['markersX'])
                aruco_marker.marker.markersY=int(a['markersY'])
                aruco_marker.marker.markerLength=float(a['markerLength'])
                aruco_marker.marker.markerSpacing=float(a['markerSpacing'])
                aruco_marker.marker.dictionary=a['dictionary']
                aruco_marker.marker.firstMarker=int(a['firstMarker'])
        
        payload_msg.markers.append(aruco_marker)
    
    #Load in meshes
    payload_mesh_visual_chain=[(None, payload_link_name)]
    if payload_link_name in urdf_robot.child_map:
        payload_mesh_visual_chain.extend(urdf_robot.child_map[payload_link_name])
    for j_name, l_name in payload_mesh_visual_chain:  
        v=urdf_robot.link_map[l_name].visual       
        j=urdf_robot.joint_map[j_name] if j_name is not None else None       
        
        if v is None: continue
        assert isinstance(v.geometry,Mesh), "Payload geometry must be a mesh, invalid payload URDF"
        assert v.geometry.scale is None, "Invalid mesh in URDF for payload, scaling not supported"
        if j is not None:
            mesh_pose=rox_msg.transform2pose_msg(rox_msg.msg2transform(_origin_to_pose(j.origin))*rox_msg.msg2transform(_origin_to_pose(v.origin)))
        else:
            mesh_pose=_origin_to_pose(v.origin)
        mesh_fname=v.geometry.filename
        payload_msg.mesh_poses.append(mesh_pose)
        payload_msg.mesh_resources.append(mesh_fname)    
    
    
    payload_msg.confidence=0.1
    
    
    #TODO: read inertial values
    
    #Sanity check
    payload_msg._check_types()
    
    return payload_msg

def xacro_string_to_payload_target(xacro_str, xacro_fname=None, **kwargs):
    
    if xacro_fname is not None:
        xacro.restore_filestack([xacro_fname])
    else:
        xacro.restore_filestack([])
    
    if 'mappings' not in kwargs:
        kwargs['mappings']={'payload_extensions': 'true'}
    
    doc = xacro.parse(xacro_str)       
    xacro.process_doc(doc, **kwargs)   
    urdf_str = doc.toprettyxml(indent='  ')
    
    return urdf_to_payload_target(urdf_str)

def urdf_to_payload_target(xml_str):
    urdf_robot = URDF.from_xml_string(xml_str)
        
    assert len(urdf_robot.link_map) == 1, "Multiple payload targets detected, invalid URDF."
    assert len(urdf_robot.joint_map) == 1, "Multiple payload targets detected, invalid URDF."
    payload_target_link=urdf_robot.link_map.itervalues().next()
    payload_target_joint=urdf_robot.joint_map.itervalues().next()
    
    assert payload_target_joint.child == payload_target_link.name, "Invalid payload target URDF"
    
    assert re.match(r'^\w+_target(?:_(\d+))?$',payload_target_link.name) is not None, \
        "Invalid payload target name in URDF file"
        
    payload_msg = PayloadTarget()        
    payload_msg.name = payload_target_link.name
    
    #Load in initial pose
    payload_msg.header.frame_id=payload_target_joint.parent    
    payload_msg.pose=_origin_to_pose(payload_target_joint.origin)    
   
    payload_msg.confidence=0.1    
     
    #Sanity check
    payload_msg._check_types()
    
    return payload_msg

def xacro_string_to_link_markers(xacro_str, xacro_fname=None, **kwargs):
    
    if xacro_fname is not None:
        xacro.restore_filestack([xacro_fname])
    else:
        xacro.restore_filestack([])
    
    if 'mappings' not in kwargs:
        kwargs['mappings']={'payload_extensions': 'true'}
    
    doc = xacro.parse(xacro_str)       
    xacro.process_doc(doc, **kwargs)   
    urdf_str = doc.toprettyxml(indent='  ')
    
    return urdf_to_link_markers(urdf_str)

def urdf_to_link_markers(xml_str):
    urdf_robot = URDF.from_xml_string(xml_str)
    urdf_et = ET.fromstring(xml_str)
    
    parent_link = None
    for _, j in urdf_robot.joint_map.items():
        if parent_link is None:
            parent_link=j.parent
        else:
            assert j.parent == parent_link, "Invalid LinkMarkers URDF, all joints must have same parent link"
            
        for _, l in urdf_robot.link_map.items():
            assert urdf_robot.parent_map[l.name][1]==parent_link, \
                "Invalid LinkMarkers URDF, all links must have same parent link"
                
    payload_msg=LinkMarkers()
    payload_msg.header.frame_id=parent_link
    
    #Load in markers    
    for _, l in urdf_robot.link_map.items():
        m = re.match(r'^\w+_marker(?:_(\d+))?$',l.name)
        assert m is not None, "Invalid LinkMarkers URDF, all links must contain Aruco tag"
        
        j = urdf_robot.joint_map[urdf_robot.parent_map[l.name][0]]
                
        pose=_origin_to_pose(j.origin)
        
        aruco_marker=ArucoGridboardWithPose()
        aruco_marker.header.frame_id = parent_link
        aruco_marker.name=l.name
        aruco_marker.pose=pose
        
        link_et=urdf_et.find('.//link[@name="' + l.name + '"]')
        marker_et=link_et.find('.//aruco_marker')
        if marker_et is not None:
            gridboard_et=marker_et.find('.//gridboard')
            if gridboard_et is not None:
                a=gridboard_et.attrib
                aruco_marker.marker.markersX=int(a['markersX'])
                aruco_marker.marker.markersY=int(a['markersY'])
                aruco_marker.marker.markerLength=float(a['markerLength'])
                aruco_marker.marker.markerSpacing=float(a['markerSpacing'])
                aruco_marker.marker.dictionary=a['dictionary']
                aruco_marker.marker.firstMarker=int(a['firstMarker'])
        
        payload_msg.markers.append(aruco_marker)
    
    
    return payload_msg
            

def urdf_to_payload_main(argv=None):
    
    parser = argparse.ArgumentParser(description="Converts a URDF file into payload msg format")
    parser.add_argument('--payload-xacro-in', type=argparse.FileType('r'), action='append', help='payload xacro file to read as input')
    parser.add_argument('--payload-target-xacro-in', type=argparse.FileType('r'), action='append', help='payload target xacro file to read as input')
    parser.add_argument('--link-markers-xacro-in', type=argparse.FileType('r'), action='append', help='link markers xacro file to read as input')
    parser_out_group=parser.add_mutually_exclusive_group(required=True)
    parser_out_group.add_argument('--pub', type=str, help="ROS topic to publish PayloadArray")
    parser_out_group.add_argument('--yaml-out', type=argparse.FileType('w'), help="File to write yaml data")
    
    args=parser.parse_known_args(argv)[0]
            
    payload_array=PayloadArray()    
    
    if args.payload_xacro_in is not None:
        for f in args.payload_xacro_in:
            with f:
                payload = xacro_string_to_payload(f.read(), f.name)            
                payload_array.payloads.append(payload)
    
    if args.payload_target_xacro_in is not None:
        for f in args.payload_target_xacro_in:
            with f:
                payload = xacro_string_to_payload_target(f.read(), f.name)            
                payload_array.payload_targets.append(payload)
    
    if args.link_markers_xacro_in is not None:
        for f in args.link_markers_xacro_in:
            with f:
                payload = xacro_string_to_link_markers(f.read(), f.name)            
                payload_array.link_markers.append(payload)
    
    if args.pub is not None:        
        rospy.init_node("urdf_to_payload", anonymous=True)
        n=rospy.Time.now()
        payload_array.header.stamp=n
        for p in payload_array.payloads:
            p.header.stamp=n
        for p in payload_array.payload_targets:
            p.header.stamp=n
        for p in payload_array.link_markers:
            p.header.stamp=n
        pub=rospy.Publisher(args.pub, PayloadArray, queue_size=100, latch=True)        
        pub.publish(payload_array)        
        rospy.spin()
        return
    
    if args.yaml_out is not None:
        with args.yaml_out as f:
            f.write(str(payload_array))
        return
    
