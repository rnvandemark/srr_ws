# If this message isn't "valid", then this data is not guaranteed to be representative of the attempt
bool valid

int8 move_direction
int8 move_type
geometry_msgs/Vector3 vehicle_linear_velocity

uint32 positions_window_size
srr_msgs/IndexedDoubleArr[4] leg_pivot_position_windows
float64[4] leg_pivot_position_averages

geometry_msgs/Vector3[4] trans_leg_pivot_to_wheel
geometry_msgs/Vector3[4] wheel_linear_velocities
float64[4] wheel_contact_angles