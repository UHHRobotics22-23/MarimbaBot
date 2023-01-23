import os
import shutil
from multiprocessing import Pool

import albumentations as A
import cv2
import numpy as np
import tqdm
from generate_data import NUM_WORKER, OUTPUT_DIR
from PIL import Image

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
        A.Affine(translate_px={"y":10, "x":10}, scale=[0.3, 1.0], rotate=[-3,3], mode=1, always_apply=True),
        A.Perspective(always_apply=True),
        A.RandomBrightnessContrast(),
        A.RandomShadow(shadow_roi=(0, 0, 1, 1), num_shadows_upper=4, shadow_dimension=8),
        A.RandomSunFlare(flare_roi=(0, 0, 1, 1), src_radius=100),
        A.PixelDropout(),
        A.RGBShift(),
        A.MedianBlur(blur_limit=3,),
        A.ZoomBlur(max_factor=1.03),
        A.Downscale(scale_min=0.6, scale_max=0.99, interpolation=cv2.INTER_NEAREST),
        ]

    augmented_path = f"{AUGMENT_OUTPUT_DIR}/{i}"
    os.makedirs(augmented_path, exist_ok=True)

    new_img = apply_transforms(img, transformations)

    Image.fromarray(new_img).save(os.path.join(augmented_path, f"staff_1.png"))
    shutil.copy(orig_txt_path, os.path.join(augmented_path, f"staff_1.txt"))
    shutil.copy(orig_ly_path, os.path.join(augmented_path, f"staff_1.ly"))

if __name__ == "__main__":
    # Call generate_sample on ids with tqdm and multiprocessing
    with Pool(NUM_WORKER) as pool:
        list(tqdm.tqdm(pool.imap(augment_sample, os.listdir(OUTPUT_DIR)), total=len(os.listdir(OUTPUT_DIR))))
