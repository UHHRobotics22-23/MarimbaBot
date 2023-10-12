std_msgs/Header header

# number of considered half-tones
int32 number_of_semitones

# readable name of lowest note in CQT
string min_note

# time between two consecutive windows (rows)
duration hop_length

# 2d vector with rows over time and step width specified by number_of_semitones
float32[] data