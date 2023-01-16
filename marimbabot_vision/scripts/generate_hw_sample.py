import sys
import os
from numpy.random import choice
from PIL import Image, ImageDraw
import re

# generates handwritten music notation from text by selecting randomly from handwritten symbols
# run like this: python generate_hw_sample.py <string to be converted>
# example: python generate_hw_sample.py "f16 g16 a16 b16 c'16 d'16 e'16 f'16 g'16 a'16 b'16 c''16 d''16 e''16 f''16 g''16 r1 r1 r2 r2 r4 r4 r4 r4 r8 r8 r16 r16 r16 r16 r2 a''8 b''8 c'''8 d'''8 e'''8 f'''8 g'''8 a'''8 g'''4 f'''4 e'''4 d'''4 c'''2 b''2 a''2 g''2 f''2 e''2 d''2 c''2 b'1 a'1 g'1 f'1 e'1 d'1"

hw_notation_dir = './hw_notation'
output_dir = './hw_data'

height = 409
width = 583 # same dimensions as data generated with generate_data.py

def draw_piece(notes):
    head_positions = {'c': [130, 95, 60, 25], 'd': [125, 90, 55, 20], 'e': [120, 85, 50, 15], 'f': [115, 80, 45, 10], 'g': [110, 75, 40, 5], 'a':[105, 70, 35, 0], 'b': [100, 65, 30, -5]}
    x_pos = 100
    y_offset = 0
    duration_counter = 0
    for (note, octave, duration) in notes:
        """check if end of page is reached"""
        if x_pos > width-60:
            x_pos = 40
            y_offset += 90
            for i in range(50, 100, 10):
                draw.line((30,i+y_offset, 550,i+y_offset), fill=(0, 0, 0, 255))

        if note == 'r':
            """select rest symbol"""
            y_pos = 50
            note_im = Image.open('%s/rest/%d/%s'%(hw_notation_dir, duration, choice(os.listdir('%s/rest/%d/'%(hw_notation_dir, duration)))))

        else:
            """select note symbol"""
            head_pos = head_positions[note][octave]
            if duration > 4 and head_pos < 70:
                """select note symbol with inwards facing flags for notes higher than g''"""
                note_im = Image.open('%s/note/%d_inv/%s'%(hw_notation_dir, duration, choice(os.listdir('%s/note/%d_inv/'%(hw_notation_dir, duration)))))
            else:
                note_im = Image.open('%s/note/%d/%s'%(hw_notation_dir, duration, choice(os.listdir('%s/note/%d/'%(hw_notation_dir, duration)))))

            if head_pos < 70:
                """rotate all notes higher than a'"""
                note_im = note_im.transpose(Image.ROTATE_180)
                y_pos = head_pos
                if head_pos < 40:
                    """draw extra lines above staff for notes higher than g''"""
                    for y in range(head_pos if head_pos%10 else head_pos+5, 45, 10):
                        extra = Image.open('%s/extra/%s'%(hw_notation_dir, choice(os.listdir('%s/extra/'%(hw_notation_dir)))))
                        im.paste(extra, (x_pos, y+y_offset), extra)
            else:
                y_pos = head_pos-30
                if head_pos > 90:
                    """draw extra lines below staff for notes lower than d'"""
                    for y in range(head_pos if head_pos%10 else head_pos-5, 85, -10):
                        extra = Image.open('%s/extra/%s'%(hw_notation_dir, choice(os.listdir('%s/extra/'%(hw_notation_dir)))))
                        im.paste(extra, (x_pos, y+y_offset), extra)
    
        """draw selected symbol and update counters"""
        im.paste(note_im, (x_pos,y_pos+y_offset), note_im)
        x_pos += 30
        duration_counter += 1/duration

        if duration_counter == 1:
            """check if current bar is full"""
            bar = Image.open('%s/bar/%s'%(hw_notation_dir, choice(os.listdir('%s/bar/'%(hw_notation_dir)))))
            im.paste(bar, (x_pos,50+y_offset), bar)
            x_pos += 20
            duration_counter = 0
            
    os.makedirs(output_dir, exist_ok=True)
    im.save('%s/1.png'%(output_dir),'PNG')

if __name__ == "__main__":
    notes = [(n[0], n.count('\''), int((re.findall(r'\d+', n)[0]))) for n in sys.argv[1].split()]

    im = Image.new('RGBA', (width, height), (255, 255, 255, 255))
    draw = ImageDraw.Draw(im)
    for i in range(50, 100, 10):
        draw.line((30,i, width-30,i), fill=(0, 0, 0, 255))
    clef = Image.open('%s/clef/%s'%(hw_notation_dir, choice(os.listdir('%s/clef/'%(hw_notation_dir)))))
    time = Image.open('%s/time/%s'%(hw_notation_dir, choice(os.listdir('%s/time/'%(hw_notation_dir)))))
    im.paste(clef, (30,30), clef)
    im.paste(time, (70,50), time)

    draw_piece(notes)