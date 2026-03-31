# Constants for the detected corner angle (degrees)
uint8 CORNER_110 = 110
uint8 CORNER_135 = 135
uint8 CORNER_160 = 160

# Header with timestamp and frame_id (lidar frame)
std_msgs/Header header

# Tool pose in the lidar frame.
# Orientation conventions:
#   CORNER_110 : tool X axis points toward the lidar
#   CORNER_135 : tool Y axis points toward the lidar
#   CORNER_160 : tool -Y axis points toward the lidar
#   Z axis always points up
geometry_msgs/Pose pose

# Visible corner angle: 110, 135 or 160
uint8 detected_corner
