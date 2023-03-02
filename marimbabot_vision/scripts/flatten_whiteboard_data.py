import argparse
import os
import re
from multiprocessing import Pool
from shutil import copyfile

import tqdm

parser = argparse.ArgumentParser(
                    prog = 'flatten_whiteboard_data.py',
                    description = 'This script generates a separate dataset including one sample folder for each captured whiteboard picture. Each sample folder contains the correspoding .txt file, the .ly file and the .png file.')

parser.add_argument('-o', '--outdir', type=str, required=True, help='The output directory for storing the whiteboard dataset')
parser.add_argument('-i', '--indir', type=str, required=True, help='The input directory containing the captured whiteoard images')
parser.add_argument('-p', '--prefix', type=str, default='staff_1', help='The prefix of the file names. Default: staff_1')
parser.add_argument('-w', '--workers', type=int, default=30, help='The number of workers. Default: 30')

def main(sample):

    sample_folder = os.path.join(args.indir, sample)
    for real_im in [i for i in os.listdir(sample_folder) if os.path.isfile(os.path.join(sample_folder,i)) and 'real' in i]:
        num = re.search(f'{args.prefix}_real_(.*).png', real_im).group(1)
        real_sample_folder = os.path.join(args.outdir, f'{sample}_{num}')
        os.makedirs(real_sample_folder, exist_ok=True)
        copyfile(os.path.join(sample_folder, f'{args.prefix}.txt'), os.path.join(real_sample_folder, f'{args.prefix}.txt'))
        copyfile(os.path.join(sample_folder, f'{args.prefix}.ly'), os.path.join(real_sample_folder, f'{args.prefix}.ly'))
        copyfile(os.path.join(sample_folder, real_im), os.path.join(real_sample_folder, f'{args.prefix}.png'))

if __name__ == "__main__":
    args = parser.parse_args()

    with Pool(args.workers) as pool:
        list(tqdm.tqdm(pool.imap(main, os.listdir(args.indir)), total=len(os.listdir(args.indir))))
