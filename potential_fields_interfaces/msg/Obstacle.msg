string frame_id
string type  # "Sphere", "Box", "Cylinder", "Ellipsoid", "Capsule", or "Mesh"
string group # "Static", "Dynamic", or "Robot"
geometry_msgs/Pose pose

# Sphere:    radius
# Cylinder:  radius, height
# Capsule:   radius, height
float32 radius
float32 height

# Box:       length, width, height
float32 length
float32 width

# Ellipsoid: semi_x, semi_y, semi_z
float32 semi_x
float32 semi_y
float32 semi_z

# Mesh:      mesh_resource, scale_x, scale_y, scale_z
string mesh_resource
float32 scale_x
float32 scale_y
float32 scale_z
