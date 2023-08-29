# Interface for the oven
std_msgs/Header header
bool oven_state # True if the oven is on, false if it is off
uint8 power_output # Percentage of power being sent to the oven
float32 target_temperature # Temperature the oven is trying to reach
float32 current_temperature # Temperature reported by the oven
