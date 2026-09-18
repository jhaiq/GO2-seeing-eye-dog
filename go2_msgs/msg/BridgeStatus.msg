# go2_msgs/msg/BridgeStatus.msg
# Observability for the hardware bridge (dry-run or physical).

std_msgs/Header header

# UNINITIALIZED | CONNECTED | DISCONNECTED | STOPPED | FAULT
string bridge_state

# Name of the concrete adapter in use, e.g. "DryRunGo2Bridge",
# "UnitreeSportBridge".
string adapter_name

# True when the adapter does not touch physical hardware.
bool dry_run

bool connected

# Age of the last accepted SafeVelocityCommand (seconds). Negative if none.
float64 last_safe_command_age_sec

# Whether the most recent transmit attempt to the adapter succeeded.
bool last_transmit_ok

uint64 transmit_attempts
uint64 transmit_failures

# Commands rejected by the bridge's own independent checks, and why.
uint64 rejected_count
string[] last_reject_reasons

# The velocity last actually handed to the adapter.
geometry_msgs/Twist last_transmitted
