#!/usr/bin/env python3

import argparse
from functools import partial
import os
import random
from multiprocessing import Pool

import tqdm
from abjad import Block, LilyPondFile, Staff, Voice
from abjad.persist import as_png
from numpy.random import choice

NUM_SAMPLES = 10000
NUM_WORKER = 24
OUTPUT_DIR = "data"
MIN_DURATION = 16 # 1/16th note

def note_sampler(duration):
    """Sample a note or rest given a duration"""
    note = random.choice(['c', 'd', 'e', 'f', 'g', 'a', 'b', 'r'])
    octave = choice(["", "'", "''"], p=[0.0, 0.8, 0.2]) if note != 'r' else ''
    return note + octave + duration

def bar_sampler():
    """Sample a bar of notes"""
    def sample_duration(durations, level=0):
        """Randomly subdivides a list of durations into smaller durations"""
        prop = 1/3
        new_durations = []
        for duration in durations: 
            if duration < args.min_duration and random.random() > prop:
                new_durations.extend(sample_duration([duration * 2, duration * 2], level + 1))
            else:
                new_durations.append(duration)
        return new_durations

    return ' '.join([note_sampler(str(duration)) for duration in sample_duration([1,])])

def generate_piece(num_bars=3):
    """Generate a piece of music"""
    string = ' '.join([bar_sampler() for _ in range(num_bars)])
    voice_1 = Voice(string, name="Voice_1")
    staff_1 = Staff([voice_1], name="Staff_1")
    return string, staff_1

def generate_sample(i, args):
    """Generate a sample and save it to disk"""
    string, staff = generate_piece()
    os.makedirs(f"{args.output_dir}/{i}", exist_ok=True)
    header_block = Block(name="header")
    header_block.tagline = "#ff"
    lilypond_file = LilyPondFile(
        items=[
            header_block,
            """#(set-default-paper-size "a8" 'landscape)""",
            "#(set-global-staff-size 16)",
            staff,
        ],
    )
    as_png(lilypond_file, f"{args.output_dir}/{i}/staff_1.png", resolution=200)
    with open(f"{args.output_dir}/{i}/staff_1.txt", 'w') as f:
        f.write(string)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Artificial data generation.")
    parser.add_argument("num_samples", metavar="N", type=int, help="Amount of data to be generated.", default=NUM_SAMPLES)
    parser.add_argument("--num_worker", type=int, required=False, help="Amount of workers that are used to generate the data.", default=NUM_WORKER)
    parser.add_argument("--min_duration", type=int, required=False, help="Minimum duration for a note, e.g. 16 for 1/16th note.", default=MIN_DURATION)
    parser.add_argument("--output_dir", type=str, required=False, help="Folder for the generated data.", default=OUTPUT_DIR)

    args = parser.parse_args()
    print(args.output_dir)

    # Call generate_sample on ids with tqdm and multiprocessing (lilypond is single threaded)
    with Pool(args.num_worker) as pool:
        list(tqdm.tqdm(pool.imap(partial(generate_sample, args=args), range(args.num_samples)), total=args.num_samples))