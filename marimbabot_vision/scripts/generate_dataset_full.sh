
#!/bin/bash

python generate_data.py 10000
python generate_hw_data.py
python generate_augmented_data.py --INPUT_DIR data_hw --AUGMENT_OUTPUT_DIR data_hw_augmented
python generate_data.py 10000
python generate_augmented_data.py