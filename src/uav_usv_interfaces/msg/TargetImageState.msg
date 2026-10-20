# Current predicted state epoch. Prediction does not renew source_stamp authority.
builtin_interfaces/Time stamp
builtin_interfaces/Time source_stamp
builtin_interfaces/Time raw_source_stamp
builtin_interfaces/Time pose_stamp
uint64 frame_sequence
uint64 mode_epoch
# Actual pixel errors and pixel derivatives at the stated current camera pose.
float64[4] state
float64[16] covariance
geometry_msgs/Quaternion body_quaternion
bool valid
string rejection_reason
string model_sha256
