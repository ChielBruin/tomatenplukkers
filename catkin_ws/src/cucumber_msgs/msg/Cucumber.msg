# Header with the time of the detection
Header header

# Flag to check in what dimension-space the message is used.
byte dimension

# The x,y and z position of the stem of the produce.
# NOTE: When 'dimension' = DIM_2D, these values should be omitted.
geometry_msgs/Vector3 stem_position

# The width and height of the produce in meter.
# When 'dimension' = 2D, these values are in pixels.
float32 width
float32 height

# The x,y position of the stem in the original image it was detected in.
int32[] image_stem_position

# the curvature of the cucumber, where 0 is straight and 1 is 90 degrees.
float32 curvature

# Possible dimensions
byte DIM_2D=2
byte DIM_3D=3
