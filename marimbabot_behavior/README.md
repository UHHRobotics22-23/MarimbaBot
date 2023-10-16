# TAMS Master Project 2022/2023 - Behavior

## 1. Description

The Behavior node acts as the central action decider and communication controller of the Marimbabot system.
It makes use of recognized commands in order to make decisions, modify a given lilypond string, calculate a corresponding Hit Sequence, and publishes its results to corresponding topics.
The included Command GUI can be used for testing the behavior by administering commands without the need of speech recognition.

### 2 Folder Structure

Following files contain the important code that determines the functionality of the Behavior:

```bash
├── launch
│   ├── marimbabot.launch  # main launch file of this submodule.
│   └── test_behavior.launch  # launch file for testing the behavior. Additionally launches the Vision node and test_speech_synthesis, and starts commandGUI.py and audio_from_ilypond.py.
└── src/marimbabot_behavior
    ├── behavior_node.py # Action decider that processes commands and publishes to topics.
    ├── interpreter.py # Calculates a Hit Sequence Message (for Planning- and Audio node) from the active lilypond string.
    ├── commandGUI.py # Graphical User Interface for easy input lilypond sequence and command admission.
  
```

## 3. Communication Network

```mermaid
flowchart TD
    A(fa:fa-microphone Speech) --> |Command| B{fa:fa-brain Behavior}
    B --> |Hit Sequence| C(fa:fa-robot Motion Planning)
    D(fa:fa-eye Vision) --> |LilyPond Sequence| B
    B --> |LilyPond Sequence| E(fa:fa-volume-uo Audio)
    B --> |Response| A
```

### 3.1 Subscriber Topics

| Topic name                     | Message Description                              | Message type                                                 |
| ------------------------------ | ------------------------------------------------ | ------------------------------------------------------------ |
| /speech_node/command           | recognized command from Speech node              | [Command.msg](marimbabot_msgs/msg/Command.msg)            |
| /vision_node/recognized_notes  | recognized lilypond sequence from Vision node    | [String.msg](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html) |
| /command_gui/note_sequence     | input lilypond sequence from the Command GUI     | [String.msg](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html) |

### 3.2 Publisher Topics

| Topic name                     | Message Description                              | Message type                                                 |
| ------------------------------ | ------------------------------------------------ | ------------------------------------------------------------ |
| /behavior_node/response        | Response for the TTS                             | [String.msg](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html) |
| /behavior_node/note_sequence   | active lilypond sequence for the command GUI     | [String.msg](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html) |

### 3.3 Action Clients

| Action name                                                      | Goal Description                                 | Goal Message type                                            |
| ---------------------------------------------------------------- | ------------------------------------------------ | ------------------------------------------------------------ |
| [HitSequenceAction](marimbabot_msgs/action/HitSequence.action)    | Hit sequence for the UR5 Motion Planning         | [HitSequence.msg](marimbabot_msgs/msg/HitSequence.msg)    |
| [LilypondAudioAction](marimbabot_msgs/action/LilypondAudio.action)| Lilypond string for the Audio Preview            | [String.msg](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html) |

## 4. Full Decision Tree

Following tree diagram showcases the different decision steps the behavior considers when processing a command.

```mermaid
flowchart TD
    B1[Is the Behavior field value 'read'?] -->|Yes| V[Receive LilyPond String from the vision module. Success?]
    V --> |Yes| H[Generate corresponding Hit Sequence. Success?]
    H --> |Yes| S1[Save LilyPond Sequence as active LilyPond sequence, generate Response: 'Notes recognized.']
    style S1 fill:#0f03,stroke:#333,stroke-width:4px
    V --> |No| F1[Generate Response: 'No notes recognized.']
    style F1 fill:#f305,stroke:#333,stroke-width:4px
    H --> |No| F1

    B1 --> |No| B2[Is the Behavior field value 'stop'?]
    B2 --> |Yes| C[Cancel all Goals.]
    style C fill:#0f03,stroke:#333,stroke-width:4px

    B2 --> |No| L[Is there an active LilyPond Sequence?]
    L --> |Yes| A1[Is there an Action field value?]
    L --> |No| F2[Generate Response: 'No notes to play.']
    style F2 fill:#f305,stroke:#333,stroke-width:4px

    A1 --> |Yes| ST[Modify the LilyPond Sequence accordingly using the default value or Parameter value if given. Success?]
    ST --> |No| F3[Generate response: 'Unable to apply changes']
    style F3 fill:#f305,stroke:#333,stroke-width:4px

    ST --> |Yes| B3[Is the Behavior field value 'play'?]
    A1 --> |No| B3
    B3 --> |Yes| PA[Is the Planning action ready?]
    PA --> |Yes| PG1[Send the active Hit sequence to the Planning action as a Goal. Success?]
    PA --> |No| F4[Generate response: 'Robot is already playing.']
    style F4 fill:#f305,stroke:#333,stroke-width:4px
    PG1 --> |Yes| A2[Is the Action field value 'loop'?]
    PG1 --> |No| F5[Send response: 'The sequence could not be played.']
    style F5 fill:#f305,stroke:#333,stroke-width:4px
    A2 --> |Yes| PG2[Keep sending the active Hit Sequence as a Goal until stopped.]
    style PG2 fill:#0f03,stroke:#333,stroke-width:4px
    A2 --> |No| S2[Done]
    style S2 fill:#0f03,stroke:#333,stroke-width:4px

    B3 --> |No| B4[Is the Behavior field value 'preview'?]
    B4 --> |Yes| LA[Is the LilyPond Audio action ready?]
    LA --> |Yes| LG1[Send the active LilyPond Sequence to the LilyPond Audio action as a Goal. Success?]
    LA --> |No| F6[Generate response: 'Preview is already playing.']
    style F6 fill:#f305,stroke:#333,stroke-width:4px
    LG1 --> |Yes| A3[Is the Action field value 'loop'?]
    LG1 --> |No| F7[Send response: 'The audio preview could not be played.']
    A3 --> |Yes| LG2[Keep sending the active LilyPond Sequence as a Goal unil stopped.]
    style LG2 fill:#0f03,stroke:#333,stroke-width:4px
    A3 --> |No| S3[Done]
    style S3 fill:#0f03,stroke:#333,stroke-width:4px

    B4 --> |No| F8[Send response: 'Command not recognized']
    style F8 fill:#f305,stroke:#333,stroke-width:4px
```
