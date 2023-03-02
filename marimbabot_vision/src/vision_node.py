#!/usr/bin/env python

import numpy as np
import rospy
import torch
from PIL import Image as PILImage
from sensor_msgs.msg import Image as ROSImage
from std_msgs.msg import String
from transformers import (DonutProcessor, VisionEncoderDecoderConfig,
                          VisionEncoderDecoderModel)

def detect_notes(img_array: np.array, pre_processor: DonutProcessor, model: VisionEncoderDecoderModel):
    # Load image
    image = PILImage.open(img_array).convert('RGB')

    # Rotate image if needed
    if image.size[0] > image.size[1]:
        image = image.transpose(PILImage.Transpose.ROTATE_90)

    # Generate initial sequence
    decoder_input_ids = pre_processor.tokenizer(
        "<s>",
        add_special_tokens=False,
        return_tensors="pt").input_ids

    # Preprocess image
    pixel_values = pre_processor(image, return_tensors="pt").pixel_values

    # Run the model
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
        return_dict_in_generate=True,
    )

    # Decode tokens
    sequence = pre_processor.batch_decode(outputs.sequences)[0][4:]

    # Clean output sequence
    strings_to_clear = [
        pre_processor.tokenizer.eos_token,
        pre_processor.tokenizer.pad_token]

    for s in strings_to_clear:
        sequence = sequence.replace(s, "")

    rospy.loginfo(sequence)

def callbackImage(data: ROSImage):    # TODO
    # http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html
    img_array = np.array(data.data).reshape((data.height, data.width))
    detect_notes(img_array)

    rospy.loginfo("received img")
    
def listener(pre_processor, model):
    rospy.init_node('vision_receiver', anonymous=True)

    rospy.Subscriber("cv_camera_node/image_raw", ROSImage, callbackImage, callback_args=(pre_processor, model))

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    MODEL = './model'

if rospy.has_param('MODEL_PATH'):
    MODEL = rospy.get_param("MODEL_PATH")

    # Load base model
    ved_config = VisionEncoderDecoderConfig.from_pretrained(MODEL)
    pre_processor = DonutProcessor.from_pretrained(MODEL)
    model = VisionEncoderDecoderModel.from_pretrained(MODEL, ignore_mismatched_sizes=True, config=ved_config)

    device = "cuda" if torch.cuda.is_available() else "cpu"
    model.to(device)

    listener(pre_processor, model)
