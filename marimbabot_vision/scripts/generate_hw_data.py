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
MIN_SYMBOL_DIST = 30
MAX_SYMBOL_DIST = 50

head_positions = {'c': 130, 'd': 125, 'e': 120, 'f': 115, 'g': 110, 'a': 105, 'b': 100}

key_sharps_order = ['f\'\'', 'c\'\'', 'g\'\'', 'd\'\'', 'a\'']
key_flats_order = ['b\'', 'e\'\'', 'a\'', 'd\'\'', 'g\'', 'c\'\'']

key_sharps_num = {'c\major': 0, 'a\minor': 0, 'g\major': 1, 'e\minor': 1, 'd\major': 2, 'b\minor': 2, 'a\major': 3, 'fs\minor': 3, 'e\major': 4, 'cs\minor': 4, 'b\major': 5, 'gs\minor': 5}
key_flats_num = {'f\major': 1, 'd\minor': 1, 'bf\major': 2, 'g\minor': 2, 'ef\major': 3, 'c\minor': 3, 'af\major': 4, 'f\minor': 4, 'df\major': 5, 'bf\minor': 5, 'gf\major': 6, 'ef\minor': 6}

def check_space(image, x_pos, y_offset, key, args):
    # check if end of page is reached
    if x_pos > SAMPLE_WIDTH-70:
        key_num = key_sharps_num[key] if key in key_sharps_num.keys() else key_flats_num[key]
        x_pos = 40 + key_num*20 + randint(args.min_symbol_dist, args.max_symbol_dist)
        y_offset += 90
        draw_staff(image, y_offset, key, args)
    return x_pos, y_offset

def get_key_accidentals(key):
    # define notes influenced by key
    key_accidentals = {'sharps': [], 'flats': []}
    if key in key_sharps_num.keys():
        key_accidentals['sharps'] = [key_note[0] for key_note in [key_sharps_order[x] for x in range(key_sharps_num[key])]]
    elif key in key_flats_num.keys():
        key_accidentals['flats'] = [key_note[0] for key_note in [key_flats_order[x] for x in range(key_flats_num[key])]]
    return key_accidentals

def check_bar(image, x_pos, y_offset, duration_counter, key, bar_accidentals, args):
    # check if current bar is full
    if duration_counter >= 1:
        draw_symbol(image, f'{args.hw_symbols_dir}/bar', (x_pos,50+y_offset))
        x_pos += randint(args.min_symbol_dist, args.max_symbol_dist)
        duration_counter = 0
        x_pos, y_offset = check_space(image, x_pos, y_offset, key, args)
        bar_accidentals = get_key_accidentals(key)
    return x_pos, y_offset, duration_counter, bar_accidentals

def get_note_pose(note, octave):
    # head_pos: y-position of the note head, y_pos: y-position of the note image
    head_pos = head_positions[note] - octave*35 
    is_flipped = head_pos < 70
    y_pos = head_pos if is_flipped else head_pos-30
    return y_pos, is_flipped 

def extend_staff(image, x_pos, y_pos, y_offset):
    # draw extra lines above staff for notes higher than g''
    draw = ImageDraw.Draw(image)
    if y_pos - y_offset <= 35:
        for y in range(y_pos+5 if y_pos%10 else y_pos+10, 50+y_offset, 10):
            draw.line((x_pos-5, y, x_pos+20, y), fill=(0, 0, 0, 255))

            # draw_symbol(image, f'{args.hw_symbols_dir}/extra', (x_pos, y+y_offset))

    # draw extra lines below staff for notes lower than d'
    elif y_pos - y_offset >= 95:
        # for y in range(y_pos+5 if y_pos%10 else y_pos+5, 100, 110):
        for y in range(y_pos+5 if y_pos%10 else y_pos+15, 90+y_offset, -10):
            draw.line((x_pos-5, y, x_pos+20, y), fill=(0, 0, 0, 255))
            # draw_symbol(image, f'{args.hw_symbols_dir}/extra', (x_pos, y+y_offset))

def draw_dynamics(image, rule, x_pos, y_pos, args):
    for symbol in rule[1:]:
        draw_symbol(image, f'{args.hw_symbols_dir}/dynamics/{symbol}', (x_pos, y_pos))
        x_pos += 15

def draw_key(image, y_offset, key, args):
    x_pos = 60
    if key in key_flats_num.keys():
        for i in range(key_flats_num[key]):
            key_flat = key_flats_order[i]
            tone = key_flat[0]
            octave = key_flat.count('\'')
            y_pos = head_positions[tone] - octave*35 + y_offset
            draw_symbol(image, f'{args.hw_symbols_dir}/accidental/flat', (x_pos, y_pos-10))
            x_pos += 20
    elif key in key_sharps_num.keys():
        for i in range(key_sharps_num[key]):
            key_sharp = key_sharps_order[i]
            tone = key_sharp[0]
            octave = key_sharp.count('\'')
            y_pos = head_positions[tone] - octave*35 + y_offset
            draw_symbol(image, f'{args.hw_symbols_dir}/accidental/sharp', (x_pos, y_pos-10))
            x_pos += 20

def draw_staff(image, y_offset, key, args):
    draw = ImageDraw.Draw(image)
    for i in range(50, 100, 10):
        draw.line((30,i+y_offset, SAMPLE_WIDTH-30,i+y_offset), fill=(0, 0, 0, 255))
    draw_symbol(image, f'{args.hw_symbols_dir}/clef', (30,30+y_offset))
    draw_key(image, y_offset, key, args)

def draw_symbol(image, symbol_path, position, is_flipped=False, is_stem=False):
    symbol_file = choice(os.listdir(symbol_path))
    symbol_im = Image.open(f'{symbol_path}/{symbol_file}')
    if is_flipped:
        symbol_im = symbol_im.transpose(Image.Transpose.ROTATE_180)
        if is_stem:
            symbol_im = symbol_im.transpose(Image.Transpose.FLIP_LEFT_RIGHT)
    image.paste(symbol_im, position, symbol_im)

def draw_note(image, position, is_flipped, duration, args):
    # choose note with inwards facing flags if higher than a'
    if duration > 4 and is_flipped:
        draw_symbol(image, f'{args.hw_symbols_dir}/note/{duration}_inv', position, is_flipped)
    else:
        draw_symbol(image, f'{args.hw_symbols_dir}/note/{duration}', position, is_flipped)

def generate_sample_image(key, tempo, args):
    image = Image.new('RGBA', (SAMPLE_WIDTH, SAMPLE_HEIGHT), (255, 255, 255, 255))
    draw_staff(image, 0, key, args)
    x_pos = 70 + (key_flats_num[key] if key in key_flats_num.keys() else key_sharps_num[key])*20
    draw_symbol(image, f'{args.hw_symbols_dir}/time', (x_pos, 50))
    if tempo:
        draw_symbol(image, f'{args.hw_symbols_dir}/tempo/{tempo[:2]}', (x_pos, 10))
        draw_symbol(image, f'{args.hw_symbols_dir}/tempo/{tempo[2:]}', (x_pos+30, 10))
    return image

def draw_piece(string, sample_name, args):
    piece = string.split()

    # get key
    key = 'c\major'
    if '\key' in piece:
        key_index = piece.index('\key')
        key = piece[key_index + 1] + piece[key_index + 2]
        piece = piece[:key_index] + piece[key_index + 3:]

    # get tempo
    tempo = None
    if '\\tempo' in piece:
        tempo_index = piece.index('\\tempo')
        tempo = piece[tempo_index + 1]
        piece = piece[:tempo_index] + piece[tempo_index + 2:]

    # initialize variables   
    x_pos = 70 + (key_flats_num[key] if key in key_flats_num.keys() else key_sharps_num[key])*20 + (30 if tempo else 0) + randint(args.min_symbol_dist, args.max_symbol_dist)
    bar_accidentals = get_key_accidentals(key)
    y_offset = 0
    duration_counter = 0
    index = 0

    # generate sample image
    sample_im = generate_sample_image(key, tempo, args)
    # piece = [(n[0], n[1], n.count('\''), int((re.findall(r'\d+', n)[0])), n.count('.')) for n in string.split()]
    
    while index < len(piece):
        rule = piece[index]

        n_indices = 0       

        # draw rest
        if rule[0] == 'r' and not rule == 'repeat':
            duration = int((re.findall(r'\d+', rule)[0]))
            dot = rule.count('.')
            draw_symbol(sample_im, f'{args.hw_symbols_dir}/rest/{duration}', (x_pos, 50 + y_offset))

            # check for dynamics
            # (necessary here, because dynamics are written behind the rest in the piece string but should appear directly underneath)
            if index < len(piece) -1 and piece[index+1][:2] in ['\\f', '\\p', '\\m']:
                draw_dynamics(sample_im, piece[index+1], x_pos-10, 90 + y_offset, args)
                n_indices += 1

            # check for accents
            # (necessary here, because accents are written behind the rest in the piece string but should appear directly above/underneath)
            if index < len(piece) -2 and piece[index+1] + piece[index+2] == '-\marcato':
                draw_symbol(sample_im, f'{args.hw_symbols_dir}/accents/marcato', (x_pos + 5, 40 + y_offset))
                n_indices += 2 

            # draw dot
            if dot >= 1:
                draw_symbol(sample_im, f'{args.hw_symbols_dir}/dot', (x_pos+20, 60 + y_offset))

            # update duration, and index counter
            duration_counter += 1/duration + dot * (0.5/duration)
            n_indices += 1

        # draw note, chord
        elif rule[0] in head_positions.keys() or rule[0] == '<':
            # check if note is chord
            if rule[0] == '<':
                rule2 = piece[index+1]
                tones = [rule[1], rule2[0]]
                accidentals = [rule[2], rule2[1]]
                octaves = [rule.count('\''), rule2.count('\'')]
                y_head_poses = [head_positions[tones[0]] - octaves[0]*35 + y_offset, head_positions[tones[1]] - octaves[1]*35 + y_offset]
                duration = int((re.findall(r'\d+', rule2)[0]))
                dot = rule2.count('.')
                # sort chord by y position (1st low note, 2nd high note)
                if y_head_poses[0] < y_head_poses[1]:
                    tones.reverse()
                    accidentals.reverse()
                    octaves.reverse()
                    y_head_poses.reverse()
                is_flipped = True if y_head_poses[0] - y_offset < 70 else False

            # non-chord note
            else:
                tones = [rule[0]]
                accidentals = [rule[1]]
                octaves = [rule.count('\'')]
                duration = int((re.findall(r'\d+', rule)[0]))
                dot = rule.count('.')
                y_head_poses = [head_positions[tones[0]] - octaves[0]*35 + y_offset]
                is_flipped = True if y_head_poses[0] - y_offset < 70 else False

            # draw accidental(s)
            for i in range(len(tones)):
                # draw flat
                if accidentals[i] == 'f':
                    if tones[i] not in bar_accidentals['flats']:
                        # prevent overlapping accidentals if notes are too close
                        if i > 0 and accidentals[i-1] in ['f', 's', 'n'] and y_head_poses[i] - y_head_poses[i-1] < 10:
                            x_pos += 20
                            draw_symbol(sample_im, f'{args.hw_symbols_dir}/accidental/flat', (x_pos, y_head_poses[i]-10))
                        bar_accidentals['flats'].append(tones[i])
                    else:
                        accidentals[i] = ''

                # draw sharp
                elif accidentals[i] == 's':
                    if tones[i] not in bar_accidentals['sharps']:
                        # prevent overlapping accidentals if notes are too close
                        if i > 0 and accidentals[i-1] in ['f', 's', 'n'] and y_head_poses[i] - y_head_poses[i-1] < 10:
                            x_pos += 20
                        draw_symbol(sample_im, f'{args.hw_symbols_dir}/accidental/sharp', (x_pos, y_head_poses[i]-10))
                        bar_accidentals['sharps'].append(tones[i])
                    else:
                        accidentals[i] = ''

                # draw natural
                elif tones[i] in bar_accidentals['flats'] or tones[i] in bar_accidentals['sharps']:
                    accidentals[i] = 'n'
                    # prevent overlapping accidentals if notes are too close
                    if i > 0 and accidentals[i-1] in ['f', 's', 'n'] and y_head_poses[i] - y_head_poses[i-1] < 10:
                        x_pos += 20
                    draw_symbol(sample_im, f'{args.hw_symbols_dir}/accidental/natural', (x_pos, y_head_poses[i]-10))
                    bar_accidentals['sharps'].remove(tones[i]) if tones[i] in bar_accidentals['sharps'] else bar_accidentals['flats'].remove(tones[i])
            
            x_pos += 20 if 'f' in accidentals or 's' in accidentals or 'n' in accidentals else 0

            # check for dynamics
            # (necessary here, because dynamics are written behind the note in the piece string but should appear directly underneath)
            if index < len(piece) -1 and piece[index+1][:2] in ['\\f', '\\p', '\\m']:
                draw_dynamics(sample_im, piece[index+1], x_pos-10, (90 + 0 if y_head_poses[0] < 80 else y_head_poses[0] + 10) + y_offset, args)
                n_indices += 1

            # check for accents
            # (necessary here, because accents are written behind the note in the piece string but should appear directly above/underneath)
            if index < len(piece) -2 and piece[index+1] + piece[index+2] == '-\marcato':
                accent_y_pos = y_head_poses[0] + 15 if not is_flipped else (y_head_poses[0] - 15 if len(tones) == 1 else y_head_poses[-1] + 15)
                draw_symbol(sample_im, f'{args.hw_symbols_dir}/accents/marcato', (x_pos+5, accent_y_pos))
                n_indices += 2

            # draw note head(s)
            for i in range(len(tones)):
                # prevent overlapping note heads if notes are too close
                note_is_flipped = is_flipped
                if i == 1 and y_head_poses[0] - y_head_poses[1] <= 5:
                    x_pos += 15
                    note_is_flipped = True
                    
                extend_staff(sample_im, x_pos, y_head_poses[i], y_offset)
                if duration < 4:
                    draw_symbol(sample_im, f'{args.hw_symbols_dir}/head/empty', (x_pos, y_head_poses[i]), note_is_flipped)
                else:
                    draw_symbol(sample_im, f'{args.hw_symbols_dir}/head/full', (x_pos, y_head_poses[i]), note_is_flipped)

            # draw note stem
            if duration > 1:
                # check if distance between note heads too small for a connecting line
                if len(tones) > 1 and y_head_poses[0] - y_head_poses[1] <= 5:
                    # draw vertical line between note heads
                    draw = ImageDraw.Draw(sample_im)
                    draw.line((x_pos, y_head_poses[0]+5, x_pos, y_head_poses[1]+5), fill=(0, 0, 0, 255), width=2)

                    attachment_point = (x_pos-10, y_head_poses[0]) if is_flipped else (x_pos-10, y_head_poses[1]-30)

                # check if distance between note heads is big enough for a connecting line
                elif len(tones) > 1 and y_head_poses[0] - y_head_poses[1] > 5:
                    # connect the note heads of the chord
                    draw = ImageDraw.Draw(sample_im)
                    if is_flipped:
                        draw.line((x_pos, y_head_poses[0]+5, x_pos, y_head_poses[1]+5), fill=(0, 0, 0, 255), width=2)
                    else:
                        draw.line((x_pos+15, y_head_poses[0]+5, x_pos+15, y_head_poses[1]+5), fill=(0, 0, 0, 255), width=2)

                    # identify attachment point for stem
                    attachment_point = (x_pos-10, y_head_poses[0]) if is_flipped else (x_pos+5, y_head_poses[1]-30)

                # handle single note
                else:
                    attachment_point = (x_pos-10, y_head_poses[0]) if is_flipped else (x_pos+5, y_head_poses[0]-30)
                
                # draw stem with correct amount of flags
                if duration <=4:
                    draw_symbol(sample_im, f'{args.hw_symbols_dir}/stem/4', attachment_point, is_flipped, True)
                else:
                    draw_symbol(sample_im, f'{args.hw_symbols_dir}/stem/{duration}', attachment_point, is_flipped, True)

            # draw dot
            if dot >= 1:
                y_pos = y_head_poses[0] - 5 if y_head_poses[0] % 10 else y_head_poses[0]
                x_pos += 25 if duration > 4 and not is_flipped and not len(tones) > 1 else 20
                draw_symbol(sample_im, f'{args.hw_symbols_dir}/dot', (x_pos, y_pos))
                if len(tones) > 1:
                    x_pos += 5 if duration > 4 and not is_flipped else 0     
                    y_pos = y_head_poses[1] - 5 if y_head_poses[1] % 10 else y_head_poses[1]
                    draw_symbol(sample_im, f'{args.hw_symbols_dir}/dot', (x_pos, y_pos))

            duration_counter += 1/duration + dot * (0.5/duration)
            n_indices += len(tones)

        else:
            print('unrecognized_symbol: ', rule)
            index += 1
            continue

        index += n_indices

        # update position, duration, and index counter
        x_pos += randint(args.min_symbol_dist, args.max_symbol_dist)

        # check if end of page is reached
        x_pos, y_offset = check_space(sample_im, x_pos, y_offset, key, args)

        # check if current bar is full
        x_pos, y_offset, duration_counter, bar_accidentals = check_bar(sample_im, x_pos, y_offset, duration_counter, key, bar_accidentals, args)
        

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
