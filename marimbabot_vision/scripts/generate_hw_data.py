import os
import tqdm
from numpy.random import choice, randint
from PIL import Image, ImageDraw
from multiprocessing import Pool
import re
import shutil

# generates handwritten music notation by selecting randomly from handwritten symbols
# iterates through the sample folders of INPUT_DIR and creates a corresponding handwritten sample folder in OUTPUT_DIR

INPUT_DIR = 'data'
OUTPUT_DIR = 'data_hw'

HW_SYMBOLS_DIR = 'hw_notation'
NAME_PREFIX = 'staff_1'

SAMPLE_HEIGHT = 409
SAMPLE_WIDTH = 583 # same dimensions as data generated with generate_data.py
MIN_SYMBOL_DIST = 20
MAX_SYMBOL_DIST = 50

head_positions = {'c': 130, 'd': 125, 'e': 120, 'f': 115, 'g': 110, 'a': 105, 'b': 100}

def check_space(image, x_pos, y_offset):
    # check if end of page is reached
    if x_pos > SAMPLE_WIDTH-50:
        x_pos = 40 + randint(MIN_SYMBOL_DIST, MAX_SYMBOL_DIST)
        y_offset += 90
        draw_staff(image, y_offset)
    return x_pos, y_offset

def check_bar(image, x_pos, y_offset, duration_counter):
    # check if current bar is full
    if duration_counter == 1:
        draw_symbol(image, f'{HW_SYMBOLS_DIR}/bar', (x_pos,50+y_offset))
        x_pos += randint(MIN_SYMBOL_DIST, MAX_SYMBOL_DIST)
        duration_counter = 0
        x_pos, y_offset = check_space(image, x_pos, y_offset)
    return x_pos, y_offset, duration_counter

def get_note_pose(note, octave):
    # head_pos: y-position of the note head, y_pos: y-position of the note image
    head_pos = head_positions[note] - octave*35 
    is_flipped = head_pos < 70
    y_pos = head_pos if is_flipped else head_pos-30
    return y_pos, is_flipped 

def extend_staff(image, x_pos, y_pos, y_offset, is_flipped):
    # draw extra lines above staff for notes higher than g''
    if y_pos < 40:
        for y in range(y_pos if y_pos%10 else y_pos+5, 45, 10):
            draw_symbol(image, f'{HW_SYMBOLS_DIR}/extra', (x_pos, y+y_offset))

    # draw extra lines below staff for notes lower than d'
    elif y_pos > 60 and not is_flipped:
        for y in range(y_pos+30 if y_pos%10 else y_pos-5, 85, -10):
            draw_symbol(image, f'{HW_SYMBOLS_DIR}/extra', (x_pos, y+y_offset))

def draw_staff(image, y_offset):
    draw = ImageDraw.Draw(image)
    for i in range(50, 100, 10):
        draw.line((30,i+y_offset, SAMPLE_WIDTH-30,i+y_offset), fill=(0, 0, 0, 255))
    draw_symbol(image, f'{HW_SYMBOLS_DIR}/clef', (30,30+y_offset))

def draw_symbol(image, symbol_path, position, is_flipped=False):
    symbol_file = choice(os.listdir(symbol_path))
    symbol_im = Image.open(f'{symbol_path}/{symbol_file}')
    if is_flipped:
        symbol_im = symbol_im.transpose(Image.Transpose.ROTATE_180)
    image.paste(symbol_im, position, symbol_im)

def draw_note(image, position, is_flipped, duration):
    # choose note with inwards facing flags if higher than a'
    if duration > 4 and is_flipped:
        draw_symbol(image, f'{HW_SYMBOLS_DIR}/note/{duration}_inv', position, is_flipped)
    else:
        draw_symbol(image, f'{HW_SYMBOLS_DIR}/note/{duration}', position, is_flipped)

def generate_sample_image():
    image = Image.new('RGBA', (SAMPLE_WIDTH, SAMPLE_HEIGHT), (255, 255, 255, 255))
    draw_staff(image, 0)
    draw_symbol(image, f'{HW_SYMBOLS_DIR}/time', (70, 50))
    return image

def draw_piece(string, sample_name):
    sample_im = generate_sample_image()
    x_pos = 70 + randint(MIN_SYMBOL_DIST, MAX_SYMBOL_DIST) 
    y_offset = 0
    duration_counter = 0
    piece = [(n[0], n.count('\''), int((re.findall(r'\d+', n)[0]))) for n in string.split()]
    for (note, octave, duration) in piece:
        x_pos, y_offset = check_space(sample_im, x_pos, y_offset)
        x_pos, y_offset, duration_counter = check_bar(sample_im, x_pos, y_offset, duration_counter)

        if note == 'r':
            draw_symbol(sample_im, f'{HW_SYMBOLS_DIR}/rest/{duration}', (x_pos,50+y_offset))
        else:
            y_pos, is_flipped = get_note_pose(note, octave)
            extend_staff(sample_im, x_pos, y_pos, y_offset, is_flipped)
            draw_note(sample_im, (x_pos, y_pos+y_offset), is_flipped, duration)
            
        x_pos += randint(MIN_SYMBOL_DIST, MAX_SYMBOL_DIST)
        duration_counter += 1/duration
    draw_symbol(sample_im, f'{HW_SYMBOLS_DIR}/bar', (x_pos,50+y_offset))

    os.makedirs(f'{OUTPUT_DIR}/{sample_name}', exist_ok=True)    
    sample_im.save(f'{OUTPUT_DIR}/{sample_name}/{NAME_PREFIX}.png','PNG')
    shutil.copyfile(f'{INPUT_DIR}/{sample_name}/{NAME_PREFIX}.txt', f'{OUTPUT_DIR}/{sample_name}/{NAME_PREFIX}.txt')
    shutil.copyfile(f'{INPUT_DIR}/{sample_name}/{NAME_PREFIX}.ly', f'{OUTPUT_DIR}/{sample_name}/{NAME_PREFIX}.ly')

def render(sample):
    with open(f'{INPUT_DIR}/{sample}/staff_1.txt', 'r') as f:
        string = f.read()
    draw_piece(string, sample)


if __name__ == "__main__":

    with Pool(32) as pool:
        list(tqdm.tqdm(pool.imap(render, os.listdir(INPUT_DIR)), total=len(os.listdir(INPUT_DIR))))
