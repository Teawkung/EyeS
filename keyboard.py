import time

from pynput.keyboard import Key, Controller



import RPi.GPIO as GPIO

def keyboardinput():

    GPIO.setmode(GPIO.BCM)

    BUTTON_GPIO = 16

    GPIO.setup(BUTTON_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    BUTTON_GPIO = 16

    keyboard = Controller()

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

                    keyboard.press('q')

                    keyboard.release('q')


            # button not pressed (or released)



            else:



                pressed = False
                
keyboardinput()
