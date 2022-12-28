import os
import random
from multiprocessing import Pool

import tqdm
from abjad import LilyPondFile, Staff, Voice
from abjad.persist import as_png
from numpy.random import choice

NUM_SAMPLES = 10000
NUM_WORKER = 24
OUTPUT_DIR = "data"

def note_sampler():
    note = random.choice(['c', 'd', 'e', 'f', 'g', 'a', 'b'])
    octave = choice(["", "'", "''"], p=[0.1, 0.8, 0.1])
    duration = random.choice(['16', '8', '4', '2', ''])
    return note + octave + duration

def generate_piece(num_notes=20):
    string = ' '.join([note_sampler() for _ in range(num_notes)])
    voice_1 = Voice(string, name="Voice_1")
    staff_1 = Staff([voice_1], name="Staff_1")
    return string, staff_1

def generate_sample(i):
    string, staff = generate_piece()
    os.makedirs(f"{OUTPUT_DIR}/{i}", exist_ok=True)
    lilypond_file = LilyPondFile(
        items=[
            """#(set-default-paper-size "a8" 'landscape)""",
            "#(set-global-staff-size 16)",
            staff,
        ],
    )
    as_png(lilypond_file, f"{OUTPUT_DIR}/{i}/staff_1.png", resolution=100)
    with open(f"{OUTPUT_DIR}/{i}/staff_1.txt", 'w') as f:
        f.write(string)

if __name__ == "__main__":
    # Call generate_sample on ids with tqdm and multiprocessing (lilypond is single threaded)
    with Pool(NUM_WORKER) as pool:
        list(tqdm.tqdm(pool.imap(generate_sample, range(NUM_SAMPLES)), total=NUM_SAMPLES))
    
