# Read-only actual Tracker clock handshake; never a control or qualification grant.
builtin_interfaces/Time stamp
string receiver
string receiver_boot_id
uint32 clock_generation
uint64 mission_id
uint64 prediction_sequence_id
uint64 active_plan_id
uint64 pending_plan_id
bool holding_qualified
bool bridge_qualified

# Receiver-owned nominal research eligibility; not a holding or execution grant.
string research_state
bool entry_allowed
bool follow_converged
float64 active_lease_deadline
float64 recovery_until
uint32 entry_attempts
uint32 valid_probe_count
string last_fallback_reason
string entry_inhibition_reason
