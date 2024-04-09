# Orientation with euler representation
# Timestamp of the data
std_msgs/Header header

# Roll-pitch-yaw in rad
float64 roll
float64 pitch
float64 yaw

# Row-major representation of the 3x3 covariance matrix
float64[9] covariance