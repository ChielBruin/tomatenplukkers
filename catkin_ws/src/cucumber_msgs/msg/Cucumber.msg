# Header with the time of the detection
Header header

# Flag to check in what dimension-space the message is used.
byte dimension

# The x,y and z position of the stem of the produce.
# NOTE: When 'dimension' = 2D, the z value should be omitted.
geometry_msgs/Vector3 stem_position

# The width and height of the produce.
# When 'dimension' = 2D, these values are between 0 and 1.
# When 'dimension' = 3D, these values are real world positions in meter.
float32 width
float32 height

# the curvature of the cucumber, where 0 is straight and 1 is 90 degrees.
float32 curvature

# Possible dimensions
byte DIM_2D=2
byte DIM_3D=2
