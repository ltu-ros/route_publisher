# A string represnting the command to follow
# Valid commands:
#   - "wait": Wait at the current location for `seconds`
#   - "goto": Drive to the location `x`,`y`

string command

# The following values are set if needed for the given command
# and are left in default state otherwise

float64 seconds
float64 x
float64 y
