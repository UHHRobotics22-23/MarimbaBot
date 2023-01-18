import sys
import os
from numpy.random import choice
from PIL import Image, ImageDraw
import re

# generates handwritten music notation from text by selecting randomly from handwritten symbols
# run like this: python generate_hw_sample.py <input dir> [optional: <output dir>]
# example: python generate_hw_sample.py ./data/1/ ./hw_data/1/
# <input dir> contains file 'staff_1.txt' which contains the lilypond text notation for this sample.
# handwritten sample and text file will be stored in <input dir> if not specified, <output dir> otherwise.  

HW_SYMBOLS_DIR = './hw_notation'
SAMPLE_HEIGHT = 409
SAMPLE_WIDTH = 583 # same dimensions as data generated with generate_data.py
SYMBOL_DIST = 30

head_positions = {'c': 130, 'd': 125, 'e': 120, 'f': 115, 'g': 110, 'a': 105, 'b': 100} # px distance from 0 to position of note head in first staff for octave=0

def check_progress(image, x_pos, y_offset, duration_counter, duration):
    # check if end of page is reached
    if x_pos > SAMPLE_WIDTH-60:
        x_pos = 40
        y_offset += 90
        draw_staff(image, y_offset)

    # check if current bar is full
    if duration_counter == 1:
        draw_symbol(image, f'{HW_SYMBOLS_DIR}/bar', (x_pos,50+y_offset))
        x_pos += SYMBOL_DIST
        duration_counter = 0

    # check if next duration is valid
    elif duration_counter + 1/duration > 1:
        print(f'given note sequence not match selected time signature!\ncurrent bar duration: {duration_counter}\nnext note duration: {duration}')
        sys.exit()

    return x_pos, y_offset, duration_counter

def get_note_pose(note, octave):
    # head_pos: y-position of the note head, y_pos: y-position of the note image
    head_pos = head_positions[note] - octave*35 
    is_flipped = head_pos < 70
    y_pos = head_pos if is_flipped else head_pos-30
    return y_pos, is_flipped 

def extend_staff(image, x_pos, y_pos, y_offset):
    # draw extra lines above staff for notes higher than g''
    if y_pos < 40:
        for y in range(y_pos if y_pos%10 else y_pos+5, 45, 10):
            draw_symbol(image, f'{HW_SYMBOLS_DIR}/extra', (x_pos, y+y_offset))

    # draw extra lines below staff for notes lower than d'
    elif y_pos > 60:
        for y in range(y_pos+30 if y_pos%10 else y_pos-5, 85, -10):
            draw_symbol(image, f'{HW_SYMBOLS_DIR}/extra', (x_pos, y+y_offset))

def draw_staff(image, y_offset):
    draw = ImageDraw.Draw(image)
    for i in range(50, 100, 10):
        draw.line((30,i+y_offset, SAMPLE_WIDTH-30,i+y_offset), fill=(0, 0, 0, 255))

def draw_symbol(image, symbol_path, position, is_flipped=False):
    symbol_file = choice(os.listdir(symbol_path))
    symbol_im = Image.open(f'{symbol_path}/{symbol_file}')
    if is_flipped:
        symbol_im = symbol_im.transpose(Image.ROTATE_180)
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
    draw_symbol(image, f'{HW_SYMBOLS_DIR}/clef', (30,30))
    draw_symbol(image, f'{HW_SYMBOLS_DIR}/time', (40+SYMBOL_DIST,50))
    return image

def draw_piece(string, outdir):
    sample_im = generate_sample_image()
    x_pos = 40+2*SYMBOL_DIST
    y_offset = 0
    duration_counter = 0
    piece = [(n[0], n.count('\''), int((re.findall(r'\d+', n)[0]))) for n in string.split()]
    for (note, octave, duration) in piece:
        x_pos, y_offset, duration_counter = check_progress(sample_im, x_pos, y_offset, duration_counter, duration)
        
        if note == 'r':
            draw_symbol(sample_im, f'{HW_SYMBOLS_DIR}/rest/{duration}', (x_pos,50+y_offset))
        else:
            y_pos, is_flipped = get_note_pose(note, octave)
            extend_staff(sample_im, x_pos, y_pos, y_offset)
            draw_note(sample_im, (x_pos, y_pos+y_offset), is_flipped, duration)
            
        x_pos += SYMBOL_DIST
        duration_counter += 1/duration

    os.makedirs(outdir, exist_ok=True)
    sample_im.save(f'{outdir}/hw_1.png','PNG')
    with open(f'{outdir}/staff_1.txt', 'w') as f:
        f.write(string)

if __name__ == "__main__":
    input_dir = sys.argv[1]
    output_dir = input_dir if len(sys.argv) < 3 else sys.argv[2]

    with open(f'{input_dir}/staff_1.txt', 'r') as f:
        string = f.read()
    draw_piece(string, output_dir)
