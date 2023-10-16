# TAMS Master Project 2022/2023 - Vision

## Scripts
For more information on the usage of the scripts, please refer [README](../marimbabot_vision/scripts/README.md).

### Data Generation
In order to generate training data for the model, the script [generate_dataset_full.sh](scripts/generate_dataset_full.sh) with a parameter N can be used. 
This will create a dataset of N samples. The dataset is saved in the `data`, `data_augmented`, `data_hw` and `data_hw_augmented` folders. Please look here [README](../marimbabot_vision/scripts/README.md#generate_dataset_fullsh) for more information on the usage of the script.

## Src
### [vision_node.py](src/vision_node.py)

This ROS node is responsible for processing images from a camera source and recognizing notes in the images using a pre-trained model. It converts the image data into a textual representation of recognized musical notes and publishes them as ROS messages.


### [visualization_node.py](src/visualization_node.py)
The ROS node receives recognized notes from the vision_node and generates visual representations of the musical notations. It uses the LilyPond library to create musical staff notation and publishes the resulting images as ROS messages for visualization.