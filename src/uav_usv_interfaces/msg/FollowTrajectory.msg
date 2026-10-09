# Dedicated FOLLOW proposal. This message never grants execution authority.
uint64 plan_id
uint64 mission_id
uint32 clock_generation
uint64 prediction_sequence_id
uint64 parent_plan_id
builtin_interfaces/Time navigation_stamp
builtin_interfaces/Time attitude_stamp
builtin_interfaces/Time observation_stamp
builtin_interfaces/Time source_stamp
builtin_interfaces/Time input_valid_until
builtin_interfaces/Time prediction_coverage_until
builtin_interfaces/Time execution_start_stamp
builtin_interfaces/Time execution_end_stamp
# Zero means no independent holding qualification.
builtin_interfaces/Time holding_valid_until
builtin_interfaces/Time published_stamp
string holding_model_id
string frame_id
string constraint_version
string camera_version
string planner_type
string validation_state
# N positive SI seconds. XYZ: N*6*3 degree-major ascending per piece, xyz per degree.
float64[] durations
float64[] xyz_coefficients
# N*4 ascending cubic coefficients, NOT scipy's descending coefficient layout.
float64[] yaw_coefficients
# Proposal diagnostics only; receiver must use local policy and independent validator.
float64[6] dynamic_limits
float64[2] fov_margins
bool nominal_sampled_valid
string reason
