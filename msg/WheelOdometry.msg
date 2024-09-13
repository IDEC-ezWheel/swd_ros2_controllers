# This message provides information about odometry communicated by the wheels via CANOpen.

std_msgs/Header header
float64 left_wheel_position  # Position of the left wheel in mm
float64 right_wheel_position # Position of the right wheel in mm
float64 baseline             # Baseline (distance between the two wheels) in meters