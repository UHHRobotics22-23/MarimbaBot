#!/usr/bin/env python3

import cv2
import os
import tqdm
import numpy as np

WORKING_DIR = "data"
VIDEO_DEVICE = "/dev/video3"
IMAGES_PER_SAMPLE = 30

vid = cv2.VideoCapture(VIDEO_DEVICE)

for sample_folder in tqdm.tqdm(sorted(os.listdir(WORKING_DIR), key= lambda x: int(x))):
    sample_folder = os.path.join(WORKING_DIR, sample_folder)
    input_file = os.path.join(sample_folder, "staff_1.png")
    output_file = os.path.join(sample_folder, "staff_1_real")
    
    if not os.path.exists(output_file + "_0.png"):
        cv2.imshow("Data collection", 

                   cv2.imread(input_file))
           # cv2.rotate(
              #  , 
              #  cv2.ROTATE_90_CLOCKWISE))

        print("Draw the shown notes on the whiteboard and press space. Press q to exit.")
        if cv2.waitKey(0) & 0xFF == ord('q'):
            break

        for i in range(IMAGES_PER_SAMPLE):
            print("This is the captured image. Press space to continue or q to exit.")
            while True:
                ret, frame = vid.read()

                cv2.imshow("Data collection cam", frame)

                if cv2.waitKey(1) & 0xFF == ord('c'):
                    break

            print(f"Saved image {i}!")
            cv2.imwrite(f"{output_file}_{i}.png", frame)

        cv2.imshow("Data collection cam", np.zeros_like(frame))

vid.release()
cv2.destroyAllWindows()