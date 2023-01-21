from generate_data import NUM_SAMPLES, NUM_WORKER, OUTPUT_DIR
from multiprocessing import Pool
from PIL import Image

import os
import tqdm
import shutil
import albumentations as A
import numpy as np
import cv2

AUGMENT_OUTPUT_DIR = "data_augmented"

def apply_transforms(img, transforms):
    """apply given transformations"""
    transform = A.Compose(transforms)
       
    transformed = transform(image=np.asarray(img))
    transformed_image = transformed["image"]

    return transformed_image

def augment_sample(i, amount = 1):
    """Generate a augmentations for a given sample and save it to disk"""
    orig_img_path = f"{OUTPUT_DIR}/{i}/staff_1.png"
    orig_txt_path = f"{OUTPUT_DIR}/{i}/staff_1.txt"
    orig_ly_path = f"{OUTPUT_DIR}/{i}/staff_1.ly"
    img = Image.open(orig_img_path)

    # set of simpler transformations
    transformations = [ 
        A.MedianBlur(blur_limit=5,), A.ZoomBlur(max_factor=1.1), \
        A.Downscale(scale_min=0.6, scale_max=0.99, interpolation=cv2.INTER_NEAREST), \
        A.Affine(translate_px={"y":10, "x":10}, rotate=[-3,3]) 
        ]

    
    augmented_path = f"{AUGMENT_OUTPUT_DIR}/{i}"
    os.makedirs(augmented_path, exist_ok=True)

    for i in range(amount):
        new_img = apply_transforms(img, transformations)

        Image.fromarray(new_img).save(os.path.join(augmented_path, f"staff_1_augment_{i}.png"))
        shutil.copy(orig_txt_path, os.path.join(augmented_path, f"staff_1_augment_{i}.txt"))
        shutil.copy(orig_ly_path, os.path.join(augmented_path, f"staff_1_augment_{i}.ly"))

if __name__ == "__main__":
    # Call generate_sample on ids with tqdm and multiprocessing
    with Pool(NUM_WORKER) as pool:
        list(tqdm.tqdm(pool.imap(augment_sample, range(NUM_SAMPLES)), total=NUM_SAMPLES))