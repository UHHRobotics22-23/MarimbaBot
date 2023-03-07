#!/usr/bin/env python

import rospy
import torch
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image as PILImage
from sensor_msgs.msg import Image as ROSImage
from transformers import (DonutProcessor, VisionEncoderDecoderConfig,
                          VisionEncoderDecoderModel)


def detect_notes(open_cv_img, pre_processor: DonutProcessor, model: VisionEncoderDecoderModel):
    # Load image
    image = PILImage.fromarray(open_cv_img).convert('RGB')

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
    sequence = pre_processor.batch_decode(outputs.sequences, skip_special_tokens=True)[0]

    rospy.logdebug(sequence)

def callbackImage(data: ROSImage, callback_args):
    rospy.logdebug("received img")

    pre_processor, model = callback_args

    # http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html
    bridge = CvBridge()

    try:
      cv_image = bridge.imgmsg_to_cv2(data)
    except CvBridgeError as e:
      rospy.logerror(e)

    detect_notes(cv_image, pre_processor, model)

    
def listener(pre_processor, model):
    rospy.init_node('vision_receiver')

    rospy.Subscriber("cv_camera_node/image_raw", ROSImage, callbackImage, callback_args=(pre_processor, model))

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    MODEL_PATH = './model'

    if rospy.has_param('MODEL_PATH'):
        MODEL_PATH = rospy.get_param("MODEL_PATH")

    # Load base model
    ved_config = VisionEncoderDecoderConfig.from_pretrained(MODEL_PATH)
    pre_processor = DonutProcessor.from_pretrained(MODEL_PATH)
    model = VisionEncoderDecoderModel.from_pretrained(MODEL_PATH, ignore_mismatched_sizes=True, config=ved_config)

    device = "cuda" if torch.cuda.is_available() else "cpu"
    model.to(device)

    listener(pre_processor, model)
