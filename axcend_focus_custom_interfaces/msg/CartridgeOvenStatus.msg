# Interface for the oven
std_msgs/Header header
bool oven_state # True if the oven is on, false if it is off
int32 power_output # Percentage of power being sent to the oven
int32 target_temperature # Temperature the oven is trying to reach
int32 current_temperature # Temperature reported by the oven
