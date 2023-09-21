#!/bin/bash

python generate_data.py 100
python generate_hw_data.py
python generate_augmented_data.py --input_dir data_hw --augment_output_dir data_hw_augmented
python generate_data.py 100
python generate_augmented_data.py