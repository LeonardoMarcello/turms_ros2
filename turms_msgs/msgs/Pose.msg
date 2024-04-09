# Pose estimation message

# Timestamp of the data
std_msgs/Header header

# Asset estimated (roll, pitch, yaw) in respect to NWU Frame
turms_msgs/Euler euler

# Position estimated (x, y, z) in NWU Frame
geometry_msgs/Point position

# Twist estimated (linear velocity, angular velocity)in body frame
geometry_msgs/Twist twist