#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import absolute_import, division, print_function

import sys
import scipy.io.wavfile as wav
import time
# import googleapiclient.discovery

from deepspeech.model import Model
import speech_recognition as sr


import rospy
# import intera_interface
import intera_interface.head_display as head

rospy.init_node("NLP")
# global limb
# limb = intera_interface.Limb('right')
# global GRIPPER
# gripper = intera_interface.Gripper('right')
# gripper.open()
headDisplay = head.HeadDisplay()




# These constants control the beam search decoder

# Beam width used in the CTC decoder when building candidate transcriptions
BEAM_WIDTH = 500

# The alpha hyperparameter of the CTC decoder. Language Model weight
LM_WEIGHT = 1.75

# The beta hyperparameter of the CTC decoder. Word insertion weight (penalty)
WORD_COUNT_WEIGHT = 1.00

# Valid word insertion weight. This is used to lessen the word insertion penalty
# when the inserted word is part of the vocabulary
VALID_WORD_COUNT_WEIGHT = 1.00

# These constants are tied to the shape of the graph used (changing them changes
# the geometry of the first layer), so make sure you use the same constants that
# were used during training

# Number of MFCC features to use
N_FEATURES = 26

# Size of the context window used for producing timesteps in the input vector
N_CONTEXT = 9


def main():
    ds = Model('./output_graph.pb', N_FEATURES, N_CONTEXT, './alphabet.txt', BEAM_WIDTH)

    r = sr.Recognizer()
    r.energy_threshold = 500

    with sr.Microphone(sample_rate=16000) as source:
        print('Say something!', file=sys.stdout)
        headDisplay.display_image("/home/team18/Grasp-Detector-master/sawyer_head/what_fruit_would_you_like.JPG")
        audio_temp = r.listen(source)
    # fs=44100

    print('Recording done!!!')

    with open("microphone-results.wav", "wb") as f:
        f.write(audio_temp.get_wav_data())
    time.sleep(1)

    fs, audio = wav.read('microphone-results.wav')

    theText = ds.stt(audio, fs)
    print(theText)

    final_value = -1
    if "av" in theText:
        final_value = 2
    elif "ap" in theText:
        final_value = 1
    elif "ba" in theText:
        final_value = 3
    elif "or" in theText:
        final_value = 5
    else:
        final_value = 7

    with open("finalvalue.txt", "wb") as f:
        f.write(str(final_value))
    time.sleep(0.5)

if __name__ == '__main__':
    main()
