import pyaudio

import wave

import time

import RPi.GPIO as GPIO

import speech_recognition as sr

import pyttsx3

import os

import time

from imutils.video import VideoStream

from imutils.video import FPS

import numpy as np

import argparse

import imutils

import time

import cv2

import math

CLASSES = [

        'background',

        'aeroplane',

        'bicycle',

        'bird',

        'boat',

        'bottle',

        'bus',

        'car',

        'cat',

        'chair',

        'cow',

        'diningtable',

        'dog',

        'horse',

        'motorbike',

        'person',

        'pottedplant',

        'sheep',

        'sofa',

        'train',

        'tvmonitor',

        ]







def objectdetection(objname):
    engine = pyttsx3.init()

    engine.setProperty('rate', 150)

    ap = argparse.ArgumentParser()

    ap.add_argument('-p', '--prototxt', required=True,

                    help="path to Caffe 'deploy' prototxt file")

    ap.add_argument('-m', '--model', required=True,

                    help='path to Caffe pre-trained model')

    ap.add_argument('-c', '--confidence', type=float, default=0.2,

                    help='minimum probability to filter weak detections'

                    )

    args = vars(ap.parse_args())



    CLASSES = [

        'background',

        'aeroplane',

        'bicycle',

        'bird',

        'boat',

        'bottle',

        'bus',

        'car',

        'cat',

        'chair',

        'cow',

        'diningtable',

        'dog',

        'horse',

        'motorbike',

        'person',

        'pottedplant',

        'sheep',

        'sofa',

        'train',

        'tvmonitor',

        ]



    COLORS = np.random.uniform(0, 0xFF, size=(len(CLASSES), 3))



    print ('[INFO] loading model...')

    net = cv2.dnn.readNetFromCaffe(args['prototxt'], args['model'])

    print ('Model information obtained')



    print ('[INFO] starting video stream...')

    vs = VideoStream(src=0).start()
    

    time.sleep(2.0)

    fps = FPS().start()

    frame_width = 600

    user_x = frame_width / 2

    user_y = frame_width

    while True:

        frame = vs.read()

        frame = imutils.resize(frame, width=frame_width)
        frame = imutils.rotate(frame, angle=180)

        (h, w) = frame.shape[:2]

        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)),

                0.007843, (300, 300), 127.5)

        net.setInput(blob)

        detections = net.forward()

        fl = False

        for i in np.arange(0, detections.shape[2]):

            confidence = detections[0, 0, i, 2]

            if confidence > args['confidence']:

                idx = int(detections[0, 0, i, 1])

                if objname in CLASSES[idx]:

                    fl = True

                    box = detections[0, 0, i, 3:7] * np.array([w, h, w,

                            h])

                    (startX, startY, endX, endY) = box.astype('int')



                    label = '{}: {:.2f}%'.format(CLASSES[idx],

                            confidence * 100)



                    cv2.rectangle(frame, (startX, startY), (endX,

                                  endY), COLORS[idx], 2)

                    y = (startY - 15 if startY - 15 > 15 else startY

                         + 15)

                    cv2.putText(

                        frame,

                        label,

                        (startX, y),

                        cv2.FONT_HERSHEY_SIMPLEX,

                        0.5,

                        COLORS[idx],

                        2,

                        )

                    centerX = (endX + startX) / 2

                    centerY = (endY + startY) / 2

                    cv2.line(frame, (int(user_x), int(user_y)),

                             (int(centerX), int(centerY)), (0xFF, 0,

                             0), 7)

                    dir = 0

                    if centerX > user_x:

                        dir = 1

                        dirr = 'right'
                        print(dirr)
                        engine.say(dirr)

                        engine.runAndWait()

                       
                    elif centerX < user_x:

                        dir = -1

                        deg = math.degrees(math.atan(abs(centerX

                                - user_x) / abs(centerY - user_y)))

                        middle = centerY - user_y

                        deg1 = centerX - user_x

                        dirr = 'left'
                        print(dirr)
                        engine.say(dirr)

                        engine.runAndWait()

                        #print("LR = " + str(deg1))
                        #print("ceta = " + str(deg))

                        """if deg1 > 50:

                            dirr = 'right'
                            print(dirr)
                            engine.say(dirr)

                            engine.runAndWait()
"""

                        if deg1 < 50 and deg1 > -50:

                            dirr = 'forward'
                            print(dirr)
                            engine.say(dirr)

                            engine.runAndWait()
                            distance = ShowDistance()
                            engine.say(distance)
                            engine.runAndWait()


                        """if deg1 < -50:

                            dirr = 'left'
                            print(dirr)
                            engine.say(dirr)

                            engine.runAndWait()
"""


        cv2.imshow('Frame', frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break       
            fps.update()

    fps.stop()
    print("[INFO] elapsed time: {:.2f}".format(fps.elapsed()))
    print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
    cv2.destroyAllWindows()
    vs.stop()
    main()



def RecordSound():

    form_1 = pyaudio.paInt16 # 16-bit resolution

    chans = 1 # 1 channel

    samp_rate = 44100 # 44.1kHz sampling rate

    chunk = 4096 # 2^12 samples for buffer

    record_secs = 3 # seconds to record

    dev_index = 2 # device index found by p.get_device_info_by_index(ii)

    wav_output_filename = 'test1.wav' # name of .wav file



    audio = pyaudio.PyAudio() # create pyaudio instantiation



    # create pyaudio stream

    stream = audio.open(format = form_1,rate = samp_rate,channels = chans, \

                        input_device_index = dev_index,input = True, \

                        frames_per_buffer=chunk)

    print("recording")

    frames = []



    # loop through stream and append audio chunks to frame array

    for ii in range(0,int((samp_rate/chunk)*record_secs)):

        data = stream.read(chunk)

        frames.append(data)



    print("finished recording")



    # stop the stream, close it, and terminate the pyaudio instantiation

    stream.stop_stream()

    stream.close()

    audio.terminate()



    # save the audio frames as .wav file

    wavefile = wave.open(wav_output_filename,'wb')

    wavefile.setnchannels(chans)

    wavefile.setsampwidth(audio.get_sample_size(form_1))

    wavefile.setframerate(samp_rate)

    wavefile.writeframes(b''.join(frames))

    wavefile.close()





def recogSound():
    r = sr.Recognizer()
    word=""
    sound = sr.AudioFile('test1.wav')

    with sound as source:

        audio = r.record(source)
        try:

            word = r.recognize_google(audio, None,'th')
        except:
            print("error")

    return word



def main():
    GPIO.setmode(GPIO.BCM)
    BUTTON_GPIO = 16
    GPIO.setup(BUTTON_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    FrameX = False

    engine = pyttsx3.init()

    engine.setProperty('rate', 150)

    engine.say('Welcome ')

    engine.runAndWait()

    BUTTON_GPIO = 16

    if __name__ == '__main__':

        GPIO.setmode(GPIO.BCM)

        GPIO.setup(BUTTON_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        pressed = False

        while True:

            # button is pressed when pin is LOW

            if not GPIO.input(BUTTON_GPIO):

                if not pressed:

                    print("Button pressed!")

                    pressed = True
                    
                    if FrameX == True:
                        FrameX = False
                        cv2.destroyAllWindows()

                        vs.stop()
                    

                    RecordSound()

                    Word = recogSound()
                    
                    if Word=="ขวดน้ำ":
                        Word="bottle"

                    if Word=="เก้าอี้":
                        Word="chair"
                    print(Word)
                    if Word in CLASSES:
                        engine.say(Word)
                        engine.runAndWait()
                        FrameX = True
                        (found, deg, dir) = objectdetection(Word)

                    else:
                        os.remove("test1.wav")

                        engine.say("Try again")
                        

                        engine.runAndWait()

            # button not pressed (or released)

            else:

                pressed = False

def ShowDistance():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    get=True
    TRIG = 20
    ECHO = 21
    print ("Distance Measurement In Progress")

    GPIO.setup(TRIG,GPIO.OUT)
    GPIO.setup(ECHO,GPIO.IN)

    GPIO.output(TRIG,False)
    print ("Waiting For Sensor To Settle")

    time.sleep(2)

    if get == True:
        GPIO.output(TRIG,True)
        time.sleep(0.00001)
        GPIO.output(TRIG,False)

        while GPIO.input(ECHO)==0:
                pulse_start = time.time()

        while GPIO.input(ECHO) ==1:
                pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start

        distance = pulse_duration * 17150
        distance = round(distance, 2)

        print ("Distance:", distance, "cm")
            

    GPIO.cleanup()
    get=False
    print(distance)
    return distance



"""engine = pyttsx3.init()

engine.setProperty('rate', 150)
(found, deg, dir) = objectdetection("bottle")"""

#ShowDistance()
main()




