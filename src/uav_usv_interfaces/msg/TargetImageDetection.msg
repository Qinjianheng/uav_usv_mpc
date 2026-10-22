# Original RGB acquisition mapped causally to ROS/system time, never receipt time.
builtin_interfaces/Time stamp
# Unmodified original Gazebo RGB acquisition epoch.
builtin_interfaces/Time raw_stamp
builtin_interfaces/Time detection_stamp
uint64 frame_sequence
uint32 image_width
uint32 image_height
# Actual original-image pixel coordinates, not normalized/letterboxed coordinates.
float64[2] center_uv
# Original-image x_min,y_min,x_max,y_max.
float64[4] bounding_box
# Row-major alternating zero/one run lengths, beginning with a zero run.
uint32[] mask_counts
float32 confidence
string class_name
bool valid
string rejection_reason
string model_sha256
