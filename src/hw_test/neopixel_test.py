import RPi.GPIO as GPIO
import traceback
import board
import time
import neopixel
import sys


def reset_neopixels(pixels):
    """
    Red and green color wipe then
    turn off all the pixels
    """
    for i in range(20):
        pixels[i] = (255, 0, 0)
        time.sleep(0.05)
    for i in range(20):
        i = 19 - i
        pixels[i] = (0, 255, 0)
        time.sleep(0.05)
    for i in range(20):
        pixels[i] = (0, 0, 0)


def start():
    # Initialize the GPIO pin w/ PWM and #of leds
    pixels = neopixel.NeoPixel(board.D12, 20)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(6, GPIO.OUT)  # set GPIO 25 as input
    GPIO.output(6, 1)

    reset_neopixels(pixels)

if __name__ == "__main__":
    start()
