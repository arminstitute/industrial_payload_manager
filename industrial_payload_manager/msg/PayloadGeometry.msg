# Geometry of payload for visual or collision
# Designed to be compatible with RViz markers, MoveIt!,
# and Tesseract

# primitives
shape_msgs/SolidPrimitive[] primitives
geometry_msgs/Pose[] primitive_poses
std_msgs/ColorRGBA[] primitive_colors # The color is optional

# meshes
string[] mesh_resources
geometry_msgs/Pose[] mesh_poses
geometry_msgs/Vector3[] mesh_scales
std_msgs/ColorRGBA[] mesh_colors # The color is optional
