# This is a ROS message definition file.
# 1. tone name (e.g. E#) -> String
# 2. octave number (e.g. 4) -> int
# 3. starting time point in seconds (e.g. 0.0 for the first note) -> ros::Time (int)
# 4. tone duration in seconds (e.g. 0.5 for a quarter note and 120 bpm) -> ros::Duration
# 5. loudness level on a scale from 0 to 1 (e.g. 0.1 for ppp, 0.4 for mp, 0.6 for mf, 0.9 for ffff) -> double / float
string tone_name
int32 octave
time start_time
duration tone_duration
float32 loudness