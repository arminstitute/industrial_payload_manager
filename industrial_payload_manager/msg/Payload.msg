
Header header

# Name of payload in MoveIt! and workspace
string name

# Current pose of payload
geometry_msgs/Pose pose

# Target pose for vacuum_gripper_tool
GripperTarget[] gripper_targets

# Payload geometry
PayloadGeometry visual_geometry
PayloadGeometry collision_geometry

# Aruco markers
ArucoGridboardWithPose[] markers

# Mass properties
geometry_msgs/Inertia inertia

# Confidence of pose estimate. Between 0 and 1
float64 confidence




