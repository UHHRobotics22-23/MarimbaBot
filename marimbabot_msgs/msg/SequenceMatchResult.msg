std_msgs/Header header
# the sequence id of the hit sequence
int32 sequence_id

# the score(recall) of the hit sequence
float32 score

# the avg time offset of the hit sequence, calculated by average the time_offset_list
float32 avg_time_offset

# the time offset of the hit sequence, calculated by the time difference between the hit plan and the audio feedback
float32[] time_offset_list

# the matched ids for the audio feedback sequence
int32[] is_matched_list

# the hit sequence of the plan
marimbabot_msgs/HitSequenceElement[] hit_sequence_elements

# the hit sequence of the audio feedback
marimbabot_msgs/NoteOnsets[] extracted_note_onsets