# TAMS Master Project 2022/2023 - Keyword Spotting

This repository work for training the model for key word spotting, it came from [Mycroft](https://github.com/MycroftAI/mycroft-precise.git) repository. And still at working.

 **Note: This is just a brief introduction for quick hand on, detailed documentation refer to official git repository of Mycroft**

# 1. Installation

Refer to official document, you can choose Source Install or Binary Install [here](https://github.com/MycroftAI/mycroft-precise#installation).

# 2. Dataset Collection

More details about dataset collection and training are [here](https://github.com/MycroftAI/mycroft-precise/wiki/Training-your-own-wake-word#how-to-train-your-own-wake-word).

1.   After installation of it, activate the virtual env with:`source .venv/bin/activate `.

2.   After the virtual environment is activated, the command `precise-collect` should be available. 

3.   Go to the folder you want to save the recorded files,  such as ``

4.   And run that command, you will see:

     ```bash
     $ precise-collect
     Audio name (Ex. recording-##): hey-computer.##
     ALSA lib pcm_dsnoop.c:638:(snd_pcm_dsnoop_open) unable to open slave
     ALSA lib pcm_dmix.c:1099:(snd_pcm_dmix_open) unable to open slave
     ALSA lib pcm_dmix.c:1099:(snd_pcm_dmix_open) unable to open slave
     Press space to record (esc to exit)...
     Recording...
     Saved as hey-computer.00.wav
     Press space to record (esc to exit)...
     ```

     -   `hey-computer`: is the file name of recorded `.wav` file, better let it to be the same of you speaking. The file will be save to the local path.

# 3. Dataset setup

The number inside of `dataset/wake_words` or `dataset/not_wake_words` means:

| FileID | voice           |
| ------ | --------------- |
| 1      | Micheal's voice |
| 2      | Juliane's voice |
| 3      | Norman's voice  |
| 4      | Tom's voice     |
| 5      | Yunlong's voice |
|        |                 |



## 3.1 Wake words

-   "hi_marimbabot" *10
-   "hello_marimbabot" *10

## 3.2 Not wake words

-   "robot"  *5
-   "hi" *5
-   "hello" *5
-   "marimba" *5
