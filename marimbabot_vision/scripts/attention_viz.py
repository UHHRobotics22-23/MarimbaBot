"""
This script is used to visualize the attention maps of the model,
showing which parts of the image are used to generate each token.
"""

import matplotlib
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.cm as cm

import numpy as np
import torch

from PIL import Image
from transformers import (DonutProcessor, VisionEncoderDecoderConfig,
                          VisionEncoderDecoderModel)

INPUT_FILE = './demo.png'
MODEL = 'Flova/omr_transformer'

# Load base model
ved_config = VisionEncoderDecoderConfig.from_pretrained(MODEL)
pre_processor = DonutProcessor.from_pretrained(MODEL)
model = VisionEncoderDecoderModel.from_pretrained(MODEL, ignore_mismatched_sizes=True, config=ved_config)

device = "cuda" if torch.cuda.is_available() else "cpu"
model.to(device)

# Load image
image = Image.open(INPUT_FILE).convert('RGB')

# Generate initial sequence
decoder_input_ids = pre_processor.tokenizer(
    "<s>",
    add_special_tokens=False,
    return_tensors="pt").input_ids

# Prepare image
pixel_values = pre_processor(image, return_tensors="pt").pixel_values

# Generate note sequence, but save the attention maps
outputs = model.generate(
    pixel_values.to(device),
    decoder_input_ids=decoder_input_ids.to(device),
    max_length=model.decoder.config.max_position_embeddings,
    early_stopping=True,
    pad_token_id=pre_processor.tokenizer.pad_token_id,
    eos_token_id=pre_processor.tokenizer.eos_token_id,
    use_cache=True,
    num_beams=1,
    bad_words_ids=[[pre_processor.tokenizer.unk_token_id]],
    output_attentions=True,
    return_dict_in_generate=True,
)

# Iterate over all generated tokens
for i, token_attention_maps in enumerate(outputs.cross_attentions):
    # Get the cross attention to the third encoder stage and take the mean over all heads
    # Reshape it to match the image aspect ratio (the attention is performed on a flattened set of tokens)
    # Also rotate the image and flip it vertically to match the input image
    mean_attention = token_attention_maps[2].mean(1).detach().cpu().numpy().reshape(19,13).T[::-1]

    # Plot both images side by side
    fig, axs = plt.subplots(1, 2, figsize=(10, 5))

    # Plot the image in the first subplot
    axs[0].imshow(image)
    axs[0].set_title("Image")

    # Plot the heatmap in the second subplot
    heatmap_plot = axs[1].imshow(mean_attention, cmap=cm.viridis, interpolation='none')
    axs[1].set_title("Attention Map")

    # Add a colorbar to the heatmap subplot
    cbar = plt.colorbar(heatmap_plot, ax=axs[1])
    cbar.set_label('Attention score')

    # Adjust spacing between subplots
    plt.tight_layout()

    # Print the current token
    print(f"Current Token: '{pre_processor.decode(outputs.sequences[0,i])}'")

    # Show the plot
    plt.show()
