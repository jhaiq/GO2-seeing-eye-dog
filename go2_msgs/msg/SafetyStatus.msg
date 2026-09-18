# go2_msgs/msg/SafetyStatus.msg
# Observability for the deterministic safety arbiter.

std_msgs/Header header

# SAFE_TO_MOVE | DEGRADED | STOPPED | EMERGENCY_STOP
string state

# Reason codes attached to the most recent decision (may be empty).
string[] reason_codes

# Age of the most recent candidate command when it was evaluated (seconds).
# Negative if no candidate has ever been received.
float64 candidate_age_sec

# The velocity actually authorized on the last decision.
geometry_msgs/Twist last_safe_command

# The candidate velocity that was last evaluated.
geometry_msgs/Twist last_candidate_command

# Number of decisions in which the arbiter modified or suppressed the
# candidate (i.e. safe command != candidate command).
uint64 intervention_count

# Total decisions evaluated.
uint64 decision_count

# Reason codes from the most recent intervention (not the most recent
# decision), so a transient cause is still visible after it clears.
string[] last_intervention_reasons

bool estop_engaged
bool hazard_context_valid
bool localization_valid
