import os
import tqdm
from multiprocessing import Pool
import re
from shutil import copyfile

NUM_WORKER = 30
INPUT_DIR = 'data'
OUTPUT_DIR = 'data_wb'
NAME_PREFIX = 'staff_1'

def main(sample):
    sample_folder = os.path.join(INPUT_DIR, sample)
    for real_im in [i for i in os.listdir(sample_folder) if os.path.isfile(os.path.join(sample_folder,i)) and 'real' in i]:
        num = re.search(f'{NAME_PREFIX}_real_(.*).png', real_im).group(1)
        real_sample_folder = os.path.join(OUTPUT_DIR, f'{sample}_{num}')
        os.makedirs(real_sample_folder, exist_ok=True)
        copyfile(os.path.join(sample_folder, f'{NAME_PREFIX}.txt'), os.path.join(real_sample_folder, f'{NAME_PREFIX}.txt'))
        copyfile(os.path.join(sample_folder, f'{NAME_PREFIX}.ly'), os.path.join(real_sample_folder, f'{NAME_PREFIX}.ly'))
        copyfile(os.path.join(sample_folder, real_im), os.path.join(real_sample_folder, real_im))
        # os.remove(os.path.join(sample_folder, real_im)) # !!! only after whiteboard data collection is finished !!!

if __name__ == "__main__":
    with Pool(NUM_WORKER) as pool:
        list(tqdm.tqdm(pool.imap(main, os.listdir(INPUT_DIR)), total=len(os.listdir(INPUT_DIR))))