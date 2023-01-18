from generate_data import NUM_SAMPLES, NUM_WORKER, OUTPUT_DIR
from multiprocessing import Pool
from PIL import Image

import tqdm
import shutil
import albumentations as A
import numpy as np
import cv2

def apply_transforms(img, transforms = [A.RGBShift()]):
    """apply given transformations"""
    transform = A.Compose(transforms)
       
    transformed = transform(image=np.asarray(img))
    transformed_image = transformed["image"]

    return transformed_image

def augment_sample(i):
    """Generate a augmentations for a given sample and save it to disk"""
    orig_img_path = f"{OUTPUT_DIR}/{i}/staff_1.png"
    orig_txt_path = f"{OUTPUT_DIR}/{i}/staff_1.txt"
    orig_ly_path = f"{OUTPUT_DIR}/{i}/staff_1.ly"
    img = Image.open(orig_img_path)

    # set of simpler transformations
    transformations = dict({
        "random_gamma": [A.RandomGamma(always_apply=True)], 
        "random_contrast": [A.RandomBrightnessContrast(always_apply=True)],
        "blur": [A.Blur(blur_limit=15, always_apply=True)],
        "median_blur" : [A.MedianBlur(blur_limit=15, always_apply=True)],
        "glass_blur": [A.GlassBlur(always_apply=True)],
        "zoom_blur": [A.ZoomBlur(always_apply=True)],
        "image_compression": [A.ImageCompression(quality_lower=20 ,always_apply=True)],
        "downscale": [A.Downscale(scale_min=0.5, scale_max=0.5, always_apply=True, interpolation=cv2.INTER_NEAREST)],
        "rotate_3": [A.Rotate(limit=[3,3], always_apply=True)],
        "rotate_6": [A.Rotate(limit=[6,6], always_apply=True)]
        })

    for key in transformations.keys():
        new_img = apply_transforms(img, transforms=transformations[key])

        Image.fromarray(new_img).save(orig_img_path.replace(".png", "") + f"_{key}.png")
        shutil.copy(orig_txt_path, orig_txt_path.replace(".txt", "") + f"_{key}.txt")
        shutil.copy(orig_ly_path, orig_ly_path.replace(".ly", "") + f"_{key}.ly")

if __name__ == "__main__":
    # Call generate_sample on ids with tqdm and multiprocessing (lilypond is single threaded)
    with Pool(NUM_WORKER) as pool:
        list(tqdm.tqdm(pool.imap(augment_sample, range(NUM_SAMPLES)), total=NUM_SAMPLES))