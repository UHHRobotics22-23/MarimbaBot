import glob
import os
from typing import List, Tuple

import numpy as np
import pytorch_lightning as pl
import torch
from nltk import edit_distance
from PIL import Image
from torch.utils.data import DataLoader, Dataset
from transformers import (DonutProcessor, VisionEncoderDecoderConfig,
                          VisionEncoderDecoderModel)

# Config
config = {"max_epochs": 500,
          "check_val_every_n_epoch": 1,
          "gradient_clip_val":1.0,
          "lr":1e-4,
          "train_batch_sizes": [24],
          "val_batch_sizes": [24],
          "num_nodes": 1,
          "warmup_steps": 300,
          "result_path": "./result",
          "verbose": True,
          "train_data_path": "data_easy/",
          "val_data_path": "test_data_easy/",
          "max_length": 50,
          "image_size": [1280//4, 960//4],
          "start_token": "<s>",
          "num_workers": 24,
          }

# Load base model

config = VisionEncoderDecoderConfig.from_pretrained("nielsr/donut-base")
config.encoder.image_size = config['image_size']
config.decoder.max_length = config['max_length']

pre_processor = DonutProcessor.from_pretrained("nielsr/donut-base")

pre_processor.image_processor.do_align_long_axis = False
pre_processor.image_processor.size = config['image_size'][::-1]

model = VisionEncoderDecoderModel.from_pretrained(
    "nielsr/donut-base", 
    ignore_mismatched_sizes=True, 
    config=config)

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
        for sample in self.dataset_lilypond_files:
            with open(sample, 'r') as f:
                ground_truth = f.read()
            self.gt_token_sequences.append(
                    ground_truth + pre_processor.tokenizer.eos_token
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
        
        image = Image.open(glob.glob(f"{os.path.dirname(sample)}/*.png")[0])

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

# Instantiate dataset and dataloader

train_dataset = NoteDataset(config['train_data_path'], max_length=config['max_length'], split="train", start_token=config['start_token'])
val_dataset = NoteDataset(config['val_data_path'], max_length=config['max_length'], split="validation", start_token=config['start_token'])

train_dataloader = DataLoader(train_dataset, batch_size=config['batch_size'][0], shuffle=True, num_workers=config['num_workers'])
val_dataloader = DataLoader(val_dataset, batch_size=config['batch_size'][0], shuffle=False, num_workers=config['num_workers'])


# Create pytorch lightning module
class DonutModelPLModule(pl.LightningModule):
    def __init__(self, config, pre_processor, model):
        super().__init__()
        self.config = config
        self.pre_processor = pre_processor
        self.model = model

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
            answer = answer.replace(self.pre_processor.tokenizer.eos_token, "").replace(config['start_token'], "")
            pred = pred.replace(self.pre_processor.tokenizer.eos_token, "").replace(config['start_token'], "")
            scores.append(edit_distance(pred, answer) / max(len(pred), len(answer)))

            if self.config.get("verbose", False) and len(scores) < 10:
                print(f"Prediction: {pred}")
                print(f"    Answer: {answer}")
                print(f" Normed ED: {scores[0]}")

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
        return train_dataloader

    def val_dataloader(self):
        return val_dataloader

# Instantiate pytorch lightning module
model_module = DonutModelPLModule(config, pre_processor, model)

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
model.save_pretrained("./model")
pre_processor.tokenizer.save_pretrained("./model")
