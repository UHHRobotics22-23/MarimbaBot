std_msgs/Header header
# Behavior is the behavior to execute, e.g. 'play'.
string behavior

# Action is the action to execute, e.g. 'decrease speed'. May be empty.
string action

# Parameter is the parameter associated with the action, e.g. '20' when using the action 'decrease speed'.
# This would decrease the speed by 20bpm.
# May be empty, then behavior node will use default setup.
string parameter