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
    "batch_size": 6,
    "num_nodes": 1,
    "verbose": True,
    "eval_data_paths": ["test_data_wb",], # "model_artificial_char_2" ],
    "start_token": "<s>",
    "num_workers": 24,
    "model": "model_artificial_char_2"#"model_9"#"model_artificial_char_2"#"./model_extended_2" #"model_char_1",
}

# Load base model

ved_config = VisionEncoderDecoderConfig.from_pretrained(config['model'])
pre_processor = DonutProcessor.from_pretrained(config['model'])
pre_processor.image_processor.do_align_long_axis = True
model = VisionEncoderDecoderModel.from_pretrained(
    config['model'],
    ignore_mismatched_sizes=True,
    config=ved_config)


# Map idenical keys so that the model isn't punished for predicting the wrong one
key_substitutions = {
    "\\key c \\major": "", # This is the default key
    "\\key a \\minor": "", # Its the relative minor of C major
    "\\key d \\major": "\\key b \\minor",
    "\\key g \\major": "\\key e \\minor",
    "\\key f \\major": "\\key d \\minor",
}


# Create dataset
class NoteDataset(Dataset):
    def __init__(
        self,
        dataset_path: str,
        max_length: int,
        split: str = "test",
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

        self.add_tokens([self.start_token])

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
test_dataset = ConcatDataset([
    NoteDataset(
        path,
        max_length=model.config.max_length,
        split="test",
        start_token=config['start_token']) for path in config['eval_data_paths']])

test_dataloader = DataLoader(test_dataset, batch_size=config['batch_size'], shuffle=False, num_workers=config['num_workers'])


# Create pytorch lightning module
class DonutModelPLModule(pl.LightningModule):
    def __init__(self, config, pre_processor, model, val_dataloader):
        super().__init__()
        self.config = config
        self.pre_processor = pre_processor
        self.model = model
        self.val_dataloader_ref = val_dataloader

    def validation_step(self, batch, batch_idx, dataset_idx=0):
        pixel_values, labels, answers = batch
        batch_size = pixel_values.shape[0]
        # we feed the prompt to the model
        decoder_input_ids = torch.full((batch_size, 1), self.model.config.decoder_start_token_id, device=self.device)

        outputs = self.model.generate(pixel_values,
                                   decoder_input_ids=decoder_input_ids,
                                   max_length=200,
                                   early_stopping=True,
                                   pad_token_id=self.pre_processor.tokenizer.pad_token_id,
                                   eos_token_id=self.pre_processor.tokenizer.eos_token_id,
                                   use_cache=True,
                                   num_beams=1,
                                   bad_words_ids=[[self.pre_processor.tokenizer.unk_token_id]],
                                   return_dict_in_generate=True,)

        predictions = []
        for seq in self.pre_processor.tokenizer.batch_decode(outputs.sequences, skip_special_tokens=True):
            predictions.append(seq)

        scores = list()
        for pred, answer in zip(predictions, answers):
            # Apply key substitutions
            for key, value in key_substitutions.items():
                pred = pred.replace(key, value)
                answer = answer.replace(key, value)
            answer = answer[:-4].strip()
            pred = pred.strip()
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

        scores = []
        for i, results in enumerate(validation_step_outputs):
            temp_scores = []
            for result in results:
                temp_scores.extend(result)
            scores.extend(temp_scores)
            self.log_dict({
                f"edit_distance_mean_{i}th_dataset": np.mean(temp_scores),
                f"edit_distance_std_{i}th_dataset": np.std(temp_scores)}, sync_dist=True)
        self.log_dict({
            "edit_distance_mean": np.mean(scores),
            "edit_distance_std": np.std(scores)}, sync_dist=True)

    def val_dataloader(self):
        return self.val_dataloader_ref

# Instantiate pytorch lightning module
model_module = DonutModelPLModule(config, pre_processor, model, test_dataloader)

# Evaluate
trainer = pl.Trainer(
    gpus=1,
    num_nodes=config['num_nodes'],
    accelerator="gpu",
    max_epochs=1,
    logger=False,
)

trainer.validate(model_module)
