#!/usr/bin/env python3

import glob
import os
from typing import List, Tuple

import numpy as np
import pytorch_lightning as pl
import torch
from nltk import edit_distance
from PIL import Image
from torch.utils.data import ConcatDataset, DataLoader, Dataset
from tqdm import tqdm
from transformers import (DonutProcessor, VisionEncoderDecoderConfig,
                          VisionEncoderDecoderModel)

# Config
config = {
    "max_epochs": 20,
    "check_val_every_n_epoch": 1,
    "gradient_clip_val":1.0,
    "lr":1e-5,
    "train_batch_sizes": [6],
    "val_batch_sizes": [6],
    "num_nodes": 1,
    "warmup_steps": 300,
    "result_path": "./result",
    "verbose": True,
    "train_data_paths": ["data_hw/", "data_augmented/", "data_wb/"],
    "val_data_paths": ["test_data/", ],
    "max_length": 40,
    "image_size": [583, 409],
    "start_token": "<s>",
    "num_workers": 24,
    "base_model": "nielsr/donut-base",
    "output_model": "./model_1",
    "add_note_vocab": True
}

# Load base model

ved_config = VisionEncoderDecoderConfig.from_pretrained(config['base_model'])
ved_config.encoder.image_size = config['image_size']
ved_config.decoder.max_length = config['max_length']

pre_processor = DonutProcessor.from_pretrained(config['base_model'])

pre_processor.image_processor.do_align_long_axis = True
pre_processor.image_processor.size["height"] = config['image_size'][0]
pre_processor.image_processor.size["width"] = config['image_size'][1]

model = VisionEncoderDecoderModel.from_pretrained(
    config['base_model'],
    ignore_mismatched_sizes=True,
    config=ved_config)

if config['add_note_vocab']:
    note_vocab = []
    notes = "cdefgab"
    octaves = ["'", "''"]
    durations = [1, 2, 4, 8, 16]
    # Add note tokens
    for note in notes:
        for octave in octaves:
            for duration in durations:
                note_vocab.append(f"{note}{octave}{duration} ")
    # Add rest tokens
    for duration in durations:
        note_vocab.append(f"r{duration} ")
else:
    note_vocab = []

model.config.pad_token_id = pre_processor.tokenizer.pad_token_id
model.config.decoder_start_token_id = pre_processor.tokenizer.convert_tokens_to_ids([config['start_token']])[0]

# Create dataset

class NoteDataset(Dataset):
    def __init__(
        self,
        dataset_path: str,
        max_length: int,
        split: str = "train",
        ignore_id: int = -100,
        start_token: str = "<s>"
    ):
        super().__init__()

        self.max_length = max_length
        self.split = split
        self.ignore_id = ignore_id
        self.start_token = start_token

        self.dataset_lilypond_files = glob.glob(f"{dataset_path}/*/*.txt")
        self.dataset_length = len(self.dataset_lilypond_files)

        self.gt_token_sequences = []
        for sample in tqdm(self.dataset_lilypond_files):
            with open(sample, 'r') as f:
                ground_truth = f.read()
            self.gt_token_sequences.append(
                    ground_truth + " " + pre_processor.tokenizer.eos_token
            )

        self.add_tokens(note_vocab + [self.start_token])

    def add_tokens(self, list_of_tokens: List[str]):
        newly_added_num = pre_processor.tokenizer.add_tokens(list_of_tokens)
        if newly_added_num > 0:
            model.decoder.resize_token_embeddings(len(pre_processor.tokenizer))

    def __len__(self) -> int:
        return self.dataset_length

    def __getitem__(self, idx: int) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        sample = self.dataset_lilypond_files[idx]

        image = Image.open(glob.glob(f"{os.path.dirname(sample)}/*.png")[0]).convert('RGB')

        pixel_values = pre_processor(image, random_padding=self.split == "train", return_tensors="pt").pixel_values
        pixel_values = pixel_values.squeeze()

        # Targets
        target_sequence = self.gt_token_sequences[idx]
        input_ids = pre_processor.tokenizer(
            target_sequence,
            add_special_tokens=False,
            max_length=self.max_length,
            padding="max_length",
            truncation=True,
            return_tensors="pt",
        )["input_ids"].squeeze(0)

        labels = input_ids.clone()
        labels[labels == pre_processor.tokenizer.pad_token_id] = self.ignore_id
        return pixel_values, labels, target_sequence

# Instantiate datasets and dataloaders

train_dataset = ConcatDataset([
    NoteDataset(
        path,
        max_length=config['max_length'],
        split="train",
        start_token=config['start_token']) for path in config['train_data_paths']])

val_dataset = ConcatDataset([
    NoteDataset(
        path,
        max_length=config['max_length'],
        split="validation",
        start_token=config['start_token']) for path in config['val_data_paths']])

train_dataloader = DataLoader(train_dataset, batch_size=config['train_batch_sizes'][0], shuffle=True, num_workers=config['num_workers'])
val_dataloader = DataLoader(val_dataset, batch_size=config['val_batch_sizes'][0], shuffle=False, num_workers=config['num_workers'])


# Create pytorch lightning module
class DonutModelPLModule(pl.LightningModule):
    def __init__(self, config, pre_processor, model, train_dataloader, val_dataloader):
        super().__init__()
        self.config = config
        self.pre_processor = pre_processor
        self.model = model
        self.train_dataloader_ref = train_dataloader
        self.val_dataloader_ref = val_dataloader

    def training_step(self, batch, batch_idx):
        pixel_values, labels, _ = batch

        outputs = self.model(pixel_values, labels=labels)
        loss = outputs.loss
        self.log_dict({"train_loss": loss}, sync_dist=True)
        return loss

    def validation_step(self, batch, batch_idx, dataset_idx=0):
        pixel_values, labels, answers = batch
        batch_size = pixel_values.shape[0]
        # we feed the prompt to the model
        decoder_input_ids = torch.full((batch_size, 1), self.model.config.decoder_start_token_id, device=self.device)

        outputs = self.model.generate(pixel_values,
                                   decoder_input_ids=decoder_input_ids,
                                   max_length=config['max_length'],
                                   early_stopping=True,
                                   pad_token_id=self.pre_processor.tokenizer.pad_token_id,
                                   eos_token_id=self.pre_processor.tokenizer.eos_token_id,
                                   use_cache=True,
                                   num_beams=1,
                                   bad_words_ids=[[self.pre_processor.tokenizer.unk_token_id]],
                                   return_dict_in_generate=True,)

        predictions = []
        for seq in self.pre_processor.tokenizer.batch_decode(outputs.sequences):
            seq = seq.replace(self.pre_processor.tokenizer.eos_token, "").replace(self.pre_processor.tokenizer.pad_token, "")
            predictions.append(seq)

        scores = list()
        for pred, answer in zip(predictions, answers):
            pred = pred.replace(self.pre_processor.tokenizer.eos_token, "")[3:]
            answer = answer.replace(self.pre_processor.tokenizer.eos_token, "")[:len(pred)]
            score = edit_distance(pred, answer)
            scores.append(score)

            if self.config.get("verbose", False) and len(scores) < 10:
                print(f"Prediction: {pred}")
                print(f"    Answer: {answer}")
                print(f" Normed ED: {score}")

        return scores

    def validation_epoch_end(self, validation_step_outputs):
        # I set this to 1 manually
        num_of_loaders = 1
        if num_of_loaders == 1:
            validation_step_outputs = [validation_step_outputs]
        assert len(validation_step_outputs) == num_of_loaders
        cnt = [0] * num_of_loaders
        total_metric = [0] * num_of_loaders
        val_metric = [0] * num_of_loaders
        for i, results in enumerate(validation_step_outputs):
            for scores in results:
                cnt[i] += len(scores)
                total_metric[i] += np.sum(scores)
            val_metric[i] = total_metric[i] / cnt[i]
            val_metric_name = f"val_metric_{i}th_dataset"
            self.log_dict({val_metric_name: val_metric[i]}, sync_dist=True)
        self.log_dict({"val_metric": np.sum(total_metric) / np.sum(cnt)}, sync_dist=True)

    def configure_optimizers(self):
        return torch.optim.Adam(self.parameters(), lr=self.config.get("lr"))

    def train_dataloader(self):
        return self.train_dataloader_ref

    def val_dataloader(self):
        return self.val_dataloader_ref

# Instantiate pytorch lightning module
model_module = DonutModelPLModule(config, pre_processor, model, train_dataloader, val_dataloader)

# Instantiate pytorch lightning trainer
trainer = pl.Trainer(
        accelerator="gpu",
        devices=1,
        max_epochs=config.get("max_epochs"),
        check_val_every_n_epoch=config.get("check_val_every_n_epoch"),
        gradient_clip_val=config.get("gradient_clip_val"),
        precision=16, # we'll use mixed precision
        num_sanity_val_steps=0,
)

# Train the model
trainer.fit(model_module)

# Save the model and tokenizer
model.save_pretrained(config['output_model'])
pre_processor.save_pretrained(config['output_model'])
ved_config.save_pretrained(config['output_model'])
