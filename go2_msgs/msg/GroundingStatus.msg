# go2_msgs/msg/GroundingStatus.msg
# Observability for multimodal caller confirmation.
#
# A blind user cannot see a terminal. This message exists so a downstream
# audio-feedback layer can always tell the user what the robot is doing,
# including the failure modes that previously manifested as a silent,
# unbounded SEARCHING state.

std_msgs/Header header

# IDLE | LISTENING | SEARCHING | CANDIDATE | CONFIRMED | TIMED_OUT | STOPPED
string state

# Why the system is in this state. One of:
#   NO_REQUEST | NO_DETECTIONS | BEARING_MISMATCH | LOW_CONFIDENCE |
#   AUDIO_STALE | ACCUMULATING | CONFIRMED | CONFIRMATION_TIMEOUT |
#   TARGET_MOVED | USER_STOP
string reason

# Scores for the best candidate on the most recent detection frame.
# -1.0 when not applicable.
float32 visual_score
float32 audio_score
float32 fused_score

# Signed angular difference between the best candidate's visual bearing and
# the acoustic bearing, in degrees, in base_link convention. NaN when audio
# is unavailable.
float32 bearing_delta_deg

bool audio_available
int32 consecutive_confirmations
int32 required_confirmations

# Seconds remaining before the current request times out. Negative when no
# request is active.
float64 request_time_remaining_sec

int32 num_detections
