# TAMS Master Project 2022/2023 - Msgs

This package contains all the custom messages and actions used in the project.

## Actions

### [HitSequence.action](action/HitSequence.action)
The HitSequence action is used to request the execution of a sequence of hits. It is defined as follows:

#### Goal Definition:

- **marimbabot_msgs/HitSequenceElement[] hit_sequence_elements**:  An array of HitSequenceElement messages that specify the hits to be played.
#### Result Definition:

- **time first_note_hit_time**: The absolute time when the first note of the sequence was hit.
- **bool success**: Indicates whether the goal was successfully achieved.
- **uint16 error_code**:
Provides an error code in case of failure

- - **uint16 SUCCESS = 0**: Successful completion.
- - **uint16 PLANNING_FAILED = 1**: Indicates that the planning process failed.
- - **uint16 EXECUTION_FAILED = 2**: Indicates that the execution of the action failed.

#### Feedback:

- **bool playing**: Indicates whether the robot is currently playing, meaning that the goal is still active.


### [LilypondAudio.action](action/LilypondAudio.action)
The LilypondAudio action is used to request the playing of an audio piece generated from a Lilypond string. It is defined as follows:

#### Goal Definition:

- **std_msgs/String lilypond_string**: A string containing Lilypond notation to be interpreted and played.
#### Result Definition:

- **bool success**: Indicates whether the audio is assumed to be played successfully.
#### Feedback:

- **bool in_progress**: Indicates whether the audio is currently being played.


## Messages

The following messages are defined in this package. For further information on the individual messages, please refer to the individual message files.

### [Command.msg](msg/Command.msg)
This message is used to send commands from the speech node to the behavior node.
Have a look at [README](../marimbabot_speech/README.md#5-command-examples) for more information on the command syntax and examples.

### [CQTStamped.msg](msg/CQTStamped.msg)
This message contains information of the Constant-Q Transform(CQT). See [README](../marimbabot_audio/README.md#4-pipeline-of-music-note-detection) for more information.
### [HitSequence.msg](msg/HitSequence.msg)
This message contains an array of HitSequenceElement messages.

### [HitSequenceElement.msg](msg/HitSequenceElement.msg)
This message contains the information single element of a HitSequence message. It includes tone information, the start time of the note, the duration of the note and the loudness of the note.

### [NoteOnset.msg](msg/NoteOnset.msg)

### [SequenceMatchResult.msg](msg/SequenceMatchResult.msg)
### [Speech.msg](msg/Speech.msg)