# Full flattened observation vector, in policy input order
float32[] observation

# Name of each component, in the order it appears in `observation`
# e.g. ["lidar", "dist_to_goal", "heading_to_goal", "wheel_speeds", "gyro"]
string[] component_names

# Starting index of each component within `observation`, same order as component_names
int32[] component_start_idx

# Length of each component within `observation`, same order as component_names
int32[] component_lengths

# Source timestamp of the data feeding each component, same order as component_names
# (assembly time as a whole is in header.stamp below)
builtin_interfaces/Time[] component_stamps

std_msgs/Header header