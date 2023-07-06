#!/usr/bin/env python3

import argparse
import os
import random
from functools import partial
from multiprocessing import Pool

import abjad
import tqdm
from abjad import Block, LilyPondFile, Staff, Voice
from abjad.persist import as_png
from numpy.random import choice

NUM_SAMPLES = 10000
NUM_WORKER = 24
OUTPUT_DIR = "data"
MIN_DURATION = 8 # 1/8th note
INCLUDE_DYNAMICS = True
INCLUDE_SLURS = False
INCLUDE_ARTICULATIONS = False
INCLUDE_SCALES = True
INCLUDE_REPEATS = True
INCLUDE_CHORDS = True


class LilypondGenerator():
    def __init__(self, include_dynamics=True, include_slurs=True, include_articulations=True,\
                  include_scales=True, include_repeat=True, include_chords=True, min_duration=8) -> None:
        self.dynamics = include_dynamics
        self.slurs = include_slurs
        self.repeat = include_repeat
        self.scale = include_scales
        self.articulations = include_articulations
        self.chords = include_chords
        self.music_notes = ['c', 'd', 'e', 'f', 'g', 'a', 'b']
        self.rests = ['r']
        self.accidentals = ['s', 'ss', 'f', 'ff']
        self.min_duration = min_duration
        # OPTIONAL: one could add more articulations to the notes
        # marcato, stopped, tenuto, staccatissimo, accent, staccato, and portato
        # http://lilypond.org/doc/v2.22/Documentation/notation/expressive-marks-attached-to-notes
        self.articulations = ['staccato', 'accent', 'tenuto', 'marcato', 'stopped', 'staccatissimo', 'portato']
        self.dynamics = ['ppp', 'pp', 'p', 'mp', 'mf', 'f', 'ff', 'fff']
        self.tempos = [40, 60, 96, 120]

    def note_sampler(self, duration, ):
        """Sample a note, dynamics or rest given a duration"""
        first_note = random.choice(self.music_notes + self.rests)
       
        octave = choice(["", "'", "''"], p=[0.0, 0.8, 0.2]) if first_note != 'r' else ''
        note = first_note + random.choice(self.accidentals) if first_note != 'r' and random.random() < 0.2 else first_note
        dot = "." if random.random() < 0.1 else ""
        retNote = note + octave

        if self.chords and random.random() < 0.1 and note != 'r':
            second_note = random.choice([n for n in self.music_notes if n != first_note])
            second_note = second_note + random.choice(self.accidentals) if random.random() < 0.2 else second_note
            retNote = "<" + note + octave + " " + second_note + octave + ">"
        return retNote + duration + dot

    def bar_sampler(self,):
        """Sample a bar of notes, rests or chords"""
        def sample_duration(durations, level=0):
            """Randomly subdivides a list of durations into smaller durations"""
            prop = 1/3
            new_durations = []
            for duration in durations: 
                if duration < self.min_duration and random.random() > prop:
                    new_durations.extend(sample_duration([duration * 2, duration * 2], level + 1))
                else:
                    new_durations.append(duration)
            return new_durations

        return ' '.join([self.note_sampler(str(duration), ) for duration in sample_duration([1,])])
    
    def add_articulation(self, voice):
        result = abjad.select(voice).leaves()
        result = result.group_by_measure()
        # random number of notes inside a part
        random_count = random.randint(2,5)
        # partition the part into groups of random_count notes
        result = result.partition_by_counts([random_count], cyclic=True)
        # flatten the groups
        parts = [abjad.select(_).flatten() for _ in result]

        # add articulations to random notes with a certain probability
        for part in parts:
            if random.random() < 0.9:
                articulation = abjad.Articulation(random.choice(self.articulations))
                abjad.attach(articulation, part[random.randint(0, len(part) - 1)])

    """
    creates a random number of ties and slurs
    code is based on the example from the abjad documentation 3.4
    """
    def add_slurs(self, voice):
        result = abjad.select(voice).leaves()
        result = result.group_by_measure()
        # random number of notes inside a part
        random_count = random.randint(2,5)
        # partition the part into groups of random_count notes
        result = result.partition_by_counts([random_count], cyclic=True)
        result.partition_by_ratio
        # flatten the groups
        parts = [abjad.select(_).flatten() for _ in result]

        # add slurs to random notes with a certain probability
        for part in parts:
            # random slur
            if random.random() < 0.9:
                # pick two random notes
                first_note_index = random.randint(0, len(part) - 2)
                second_note_index =random.randint(first_note_index + 1, len(part) - 1)
                first_note, last_note = part[first_note_index], part[second_note_index]
                start_slur = abjad.StartSlur()
                abjad.attach(start_slur, first_note)
                stop_slur = abjad.StopSlur()
                abjad.attach(stop_slur, last_note)

        return voice

    """
    adds a repeat to the voice
    """
    def add_repeat(self, voice):
        if random.random() < 0.5:
            repeat = abjad.Repeat()
            abjad.attach(repeat, voice)

    """
    adds a tempo to the voice
    """
    def add_tempo(self, voice):
        if random.random() < 0.3:
            abjad.attach(abjad.MetronomeMark((1, 4), random.choice(self.tempos)), voice[0])

    """
    adds a major key signature to the voice, i.e. pitch
    """
    def add_major_minor_scales(self, voice):
        key_signature = abjad.KeySignature(
        abjad.NamedPitchClass(random.choice(self.music_notes)), abjad.Mode(random.choice(["major", "minor"]))
        )
        abjad.attach(key_signature, voice[0])

    """
    adds dynamics to the voice
    """
    def add_dynamics(self, voice):
        if random.random() < 0.3:
            abjad.attach(abjad.Dynamic(random.choice(self.dynamics)), voice[0])

    """
    generates a piece of music
    """
    def generate_piece(self,num_bars=3,):
        """Generate a piece of music"""
        string = ' '.join([self.bar_sampler() for _ in range(num_bars)])

        voice_1 = Voice(string, name="Voice_1")
        self.add_tempo(voice_1)

        if self.slurs:
            self.add_slurs(voice_1)
        if self.articulations:
            self.add_articulation(voice_1)
        if self.repeat:
            self.add_repeat(voice_1)
        if self.scale:
            self.add_major_minor_scales(voice_1)
        if self.dynamics:
            self.add_dynamics(voice_1)

        staff_1 = Staff([voice_1], name="Staff_1")

        # as the lilypond data lies inside brackets we need to remove them to get a clean string
        # e.g. '\\context Voice = "Voice_1"\n{\n    \\tempo 4=60\n    c\'8\n    d\'8\n    e\'8\n    f\'8\n}'
        string = abjad.lilypond(voice_1)
        # the following statement  gets rid of the context and the newlines
        string = " ".join(string.replace("\\context Voice = \"Voice_1\"\n{", "").replace("\n", "")[:-1].split())

        return string, staff_1

def generate_sample(i, args):
    """Generate a sample and save it to disk"""
    lilypondGenerator = args.lilypondGenerator
    string, staff = lilypondGenerator.generate_piece()
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
    parser.add_argument("--dynamics", type=bool, required=False, help="Determine whether to sample data that includes dynamics.", default=INCLUDE_DYNAMICS)
    parser.add_argument("--slurs", type=bool, required=False, help="Determine whether to sample data that includes dynamics slurs.", default=INCLUDE_SLURS)
    parser.add_argument("--scales", type=bool, required=False, help="Determine whether to sample data that includes dynamics scales.", default=INCLUDE_SCALES)
    parser.add_argument("--articulations", type=bool, required=False, help="Determine whether to sample data that includes dynamics articulations.", default=INCLUDE_ARTICULATIONS)
    parser.add_argument("--chords", type=bool, required=False, help="Determine whether to sample data that includes dynamics chords.", default=INCLUDE_CHORDS)
    parser.add_argument("--repeats", type=bool, required=False, help="Determine whether to sample data that includes dynamics repeats.", default=INCLUDE_REPEATS)


    args = parser.parse_args()
    args.lilypondGenerator = LilypondGenerator(include_dynamics=args.dynamics, include_articulations=args.articulations,\
                                               include_chords=args.chords, include_scales=args.scales,\
                                                include_repeat=args.repeats, include_slurs=args.slurs, min_duration=args.min_duration)
    print(args.output_dir)

    generate_sample(0, args)

    # Call generate_sample on ids with tqdm and multiprocessing (lilypond is single threaded)
    with Pool(args.num_worker) as pool:
        list(tqdm.tqdm(pool.imap(partial(generate_sample, args=args), range(args.num_samples)), total=args.num_samples))
