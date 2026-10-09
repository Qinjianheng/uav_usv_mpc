# Only the actual trajectory_tracker_node publishes this acknowledgement.
builtin_interfaces/Time stamp
uint64 plan_id
uint64 mission_id
uint32 clock_generation
uint64 active_plan_id
uint64 pending_plan_id
string receiver
# REJECTED / ACCEPTED / ACTIVE / REVOKED / EXPIRED. ACCEPTED is not a replacement.
string state
string[] reasons
# Current implementation always ORIGINAL_FOLLOW: holding and initial bridge unqualified.
string control_owner
bool replaced

# Actual receiver boot identity and independent planner process identity.
string receiver_boot_id
string planner_boot_id
