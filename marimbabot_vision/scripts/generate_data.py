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
            if duration < MIN_DURATION and random.random() > prop:
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

def generate_sample(i):
    """Generate a sample and save it to disk"""
    string, staff = generate_piece()
    os.makedirs(f"{OUTPUT_DIR}/{i}", exist_ok=True)
    lilypond_file = LilyPondFile(
        items=[
            """#(set-default-paper-size "a8" 'landscape)""",
            "#(set-global-staff-size 16)",
            staff,
        ],
    )
    as_png(lilypond_file, f"{OUTPUT_DIR}/{i}/staff_1.png", resolution=200)
    with open(f"{OUTPUT_DIR}/{i}/staff_1.txt", 'w') as f:
        f.write(string)

if __name__ == "__main__":
    # Call generate_sample on ids with tqdm and multiprocessing (lilypond is single threaded)
    with Pool(NUM_WORKER) as pool:
        list(tqdm.tqdm(pool.imap(generate_sample, range(NUM_SAMPLES)), total=NUM_SAMPLES))
  
