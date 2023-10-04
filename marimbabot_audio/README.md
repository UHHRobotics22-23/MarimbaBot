# TAMS Master Project 2022/2023 - Music Information Retrieval(Audio)

## 1. Motivation

The robot's performance of playing the marimba need to be evaluated. Therefore, this submodule is designed to can evaluate the music produced by robot through the audio feedback. It can detect the western music note by the raw audio input, and visualize it in form of midi figure and  Constant-Q transform spectrum, also there is a final score for evaluate the final motion.

## 2. Overview

### 2.1 Folder overview

The following folder tree structure show some important files, unimportant files are ignored.

```bash
├── launch
│   ├── audio_feedback.launch  # To lauch the music note detector, you can tune the parametes here.
│   ├── marimbabot.launch  # main launch file of this submodule.
│   └── open_rviz.launch  # the launch file the open the rviz.
└── src
    ├── audio_from_lilypond.py  # music systhesis,  test music sequence to audio music 
    ├── eval_visualization.py  # midi visualiazation for evaluation, the mismatch the note wil be shown here.
    ├── onset_detection.py  # onset detection and music note classification, also spectrum visualization.
    ├── onsets_visualization.py  #  the live midi visualization.
    └── sequence_evaluation.py  # music evaluation, compare the groundtruth with the robot performance.
```

### 2.2 Node Overview

<img src="./README.assets/image-20231004133827966.png" alt="image-20231004133827966" />