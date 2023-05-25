# This is a ROS message definition file.
# The following variables are defined as in https://github.com/UHHRobotics22-23/marimbabot/issues/111#issue-1706376396
# and their explanations are copied from there.
# 1. tone name(s) + octave(s) (e.g. c', <d' g'>) -> String
# 2. starting time point in seconds (e.g. 0.0 for the first note) -> ros::Time (int)
# 3. tone duration in seconds (e.g. 0.5 for a quarter note and 120 bpm) -> ros::Duration
# 4. loudness level on a scale from 0 to 1 (e.g. 0.1 for ppp, 0.4 for mp, 0.6 for mf, 0.9 for ffff) -> double / float
std_msgs/String tone_name
std_msgs/Time start_time
std_msgs/Duration tone_duration
float32 loudness