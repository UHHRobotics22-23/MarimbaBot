#!/usr/bin/env python3

import argparse
from functools import partial
import os
import re
import shutil
from multiprocessing import Pool

import tqdm
from numpy.random import choice, randint
from PIL import Image, ImageDraw

# use import for default value
from generate_data import OUTPUT_DIR as INPUT_DIR

# generates handwritten music notation by selecting randomly from handwritten symbols
# iterates through the sample folders of INPUT_DIR and creates a corresponding handwritten sample folder in OUTPUT_DIR


NUM_WORKER = 32

HW_SYMBOLS_DIR = 'hw_notation'
NAME_PREFIX = 'staff_1'
OUTPUT_DIR = 'data_hw'

SAMPLE_HEIGHT = 409
SAMPLE_WIDTH = 583 # same dimensions as data generated with generate_data.py
MIN_SYMBOL_DIST = 20
MAX_SYMBOL_DIST = 50

head_positions = {'c': 130, 'd': 125, 'e': 120, 'f': 115, 'g': 110, 'a': 105, 'b': 100}

def check_space(image, x_pos, y_offset, args):
    # check if end of page is reached
    if x_pos > SAMPLE_WIDTH-50:
        x_pos = 40 + randint(args.min_symbol_dist, args.max_symbol_dist)
        y_offset += 90
        draw_staff(image, y_offset, args)
    return x_pos, y_offset

def check_bar(image, x_pos, y_offset, duration_counter, args):
    # check if current bar is full
    if duration_counter == 1:
        draw_symbol(image, f'{args.hw_symbols_dir}/bar', (x_pos,50+y_offset))
        x_pos += randint(args.min_symbol_dist, args.max_symbol_dist)
        duration_counter = 0
        x_pos, y_offset = check_space(image, x_pos, y_offset, args)
    return x_pos, y_offset, duration_counter

def get_note_pose(note, octave):
    # head_pos: y-position of the note head, y_pos: y-position of the note image
    head_pos = head_positions[note] - octave*35 
    is_flipped = head_pos < 70
    y_pos = head_pos if is_flipped else head_pos-30
    return y_pos, is_flipped 

def extend_staff(image, x_pos, y_pos, y_offset, is_flipped, args):
    # draw extra lines above staff for notes higher than g''
    if y_pos < 40:
        for y in range(y_pos if y_pos%10 else y_pos+5, 45, 10):
            draw_symbol(image, f'{args.hw_symbols_dir}/extra', (x_pos, y+y_offset))

    # draw extra lines below staff for notes lower than d'
    elif y_pos > 60 and not is_flipped:
        for y in range(y_pos+30 if y_pos%10 else y_pos-5, 85, -10):
            draw_symbol(image, f'{args.hw_symbols_dir}/extra', (x_pos, y+y_offset))

def draw_staff(image, y_offset, args):
    draw = ImageDraw.Draw(image)
    for i in range(50, 100, 10):
        draw.line((30,i+y_offset, SAMPLE_WIDTH-30,i+y_offset), fill=(0, 0, 0, 255))
    draw_symbol(image, f'{args.hw_symbols_dir}/clef', (30,30+y_offset))

def draw_symbol(image, symbol_path, position, is_flipped=False):
    symbol_file = choice(os.listdir(symbol_path))
    symbol_im = Image.open(f'{symbol_path}/{symbol_file}')
    if is_flipped:
        symbol_im = symbol_im.transpose(Image.Transpose.ROTATE_180)
    image.paste(symbol_im, position, symbol_im)

def draw_note(image, position, is_flipped, duration, args):
    # choose note with inwards facing flags if higher than a'
    if duration > 4 and is_flipped:
        draw_symbol(image, f'{args.hw_symbols_dir}/note/{duration}_inv', position, is_flipped)
    else:
        draw_symbol(image, f'{args.hw_symbols_dir}/note/{duration}', position, is_flipped)

def generate_sample_image(args):
    image = Image.new('RGBA', (SAMPLE_WIDTH, SAMPLE_HEIGHT), (255, 255, 255, 255))
    draw_staff(image, 0, args)
    draw_symbol(image, f'{args.hw_symbols_dir}/time', (70, 50))
    return image

def draw_piece(string, sample_name, args):
    sample_im = generate_sample_image(args)
    x_pos = 70 + randint(args.min_symbol_dist, args.max_symbol_dist) 
    y_offset = 0
    duration_counter = 0
    piece = [(n[0], n[1], n.count('\''), int((re.findall(r'\d+', n)[0])), n.count('.')) for n in string.split()]

    for (note, accidental, octave, duration, dot) in piece:
        x_pos, y_offset = check_space(sample_im, x_pos, y_offset, args)
        x_pos, y_offset, duration_counter = check_bar(sample_im, x_pos, y_offset, duration_counter, args)

        if note == 'r':
            draw_symbol(sample_im, f'{args.hw_symbols_dir}/rest/{duration}', (x_pos,50+y_offset))
        else:
            y_pos, is_flipped = get_note_pose(note, octave)
            y_head_pos = head_positions[note]-octave*35+y_offset

            if accidental == 'f':
                draw_symbol(sample_im, f'{args.hw_symbols_dir}/accidental/flat', (x_pos, y_head_pos-5))
                x_pos += 20
            elif accidental == 's':
                draw_symbol(sample_im, f'{args.hw_symbols_dir}/accidental/sharp', (x_pos, y_head_pos-5))
                x_pos += 20

            extend_staff(sample_im, x_pos, y_pos, y_offset, is_flipped, args)

            draw_note(sample_im, (x_pos, y_pos+y_offset), is_flipped, duration, args)

            if dot == 1:
                draw_symbol(sample_im, f'{args.hw_symbols_dir}/dot', (x_pos+10, y_head_pos))
                x_pos += 20
            
        x_pos += randint(args.min_symbol_dist, args.max_symbol_dist)
        duration_counter += 1/duration
    draw_symbol(sample_im, f'{args.hw_symbols_dir}/bar', (x_pos,50+y_offset))

    os.makedirs(f'{args.output_dir}/{sample_name}', exist_ok=True)    
    sample_im.convert('RGB').save(f'{args.output_dir}/{sample_name}/{args.name_prefix}.png','PNG')
    shutil.copyfile(f'{args.input_dir}/{sample_name}/{args.name_prefix}.txt', f'{args.output_dir}/{sample_name}/{args.name_prefix}.txt')
    shutil.copyfile(f'{args.input_dir}/{sample_name}/{args.name_prefix}.ly', f'{args.output_dir}/{sample_name}/{args.name_prefix}.ly')

def render(sample, args):
    with open(f'{args.input_dir}/{sample}/staff_1.txt', 'r') as f:
        string = f.read()
    draw_piece(string, sample, args)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Hand-written data generation.")
    parser.add_argument("--hw_symbols_dir", type=str, required=False, help="Folder for the generated data.", default=HW_SYMBOLS_DIR)
    parser.add_argument("--num_worker", type=int, required=False, help="Amount of workers that are used to generate the data.", default=NUM_WORKER)
    parser.add_argument("--name_prefix", type=str, required=False, help="Name prefix for the generated output files.", default=NAME_PREFIX)
    parser.add_argument("--min_symbol_dist", type=int, required=False, help="Minimum distance between notes.", default=MIN_SYMBOL_DIST)
    parser.add_argument("--max_symbol_dist", type=int, required=False, help="Maximum distance between notes.", default=MAX_SYMBOL_DIST)
    parser.add_argument("--input_dir", type=str, required=False, help="Folder for the input data.", default=INPUT_DIR)
    parser.add_argument("--output_dir", type=str, required=False, help="Folder for the output data.", default=OUTPUT_DIR)

    args = parser.parse_args()

    with Pool(args.num_worker) as pool:
        list(tqdm.tqdm(pool.imap(partial(render, args=args), os.listdir(args.input_dir)), total=len(os.listdir(args.input_dir))))
