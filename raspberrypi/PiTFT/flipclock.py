# flipclock.py
#
# Display flipclock-like display on PiTFT.

import collections
from datetime import datetime
import os
import time

import digitalio
import board
import numpy as np
from PIL import Image, ImageDraw
from adafruit_rgb_display import ili9341

# Figure directory of script, to find jpg resources.
work_dir = os.path.dirname(os.path.realpath(__file__))

# Setup display - 2.8" SPI.

cs_pin = digitalio.DigitalInOut(board.CE0)
dc_pin = digitalio.DigitalInOut(board.D25)
reset_pin = digitalio.DigitalInOut(board.D24)
BAUDRATE = 24000000

spi = board.SPI()
disp = ili9341.ILI9341(spi, rotation=270, cs=cs_pin, dc=dc_pin, rst=reset_pin,
                       baudrate=BAUDRATE)
# Rotation 90 means we swap width and height.
height = disp.width
width = disp.height
image = Image.new("RGB", (width, height))


# Extract the individual images from the big pixmap.
def setup_digit_images_c5():
    image = Image.open(os.path.join(work_dir, 'COUNTDOWN_TIMER_5.jpg'))
    digits = [[]] * 10
    for digit in range(10):
        x = 310 + digit * 449
        y = 2762
        w = 344
        h = 580
        im = image.crop((x, y, x + w, y + h))
        digits[(digit + 1) % 10] = im.resize((70, 120), Image.BICUBIC)
    return digits, {}

def colorize_image(image):
    """Apply an orange cast to an image."""
    # (RGB) at 1.0, 0.8, 0.6 makes it kind of white.
    # Further reduce G and B (0.5, 0.2) for orange.
    im_array = np.array(image.getdata())  # R is at 100%
    im_array[:, 1] = 0.8 * im_array[:, 1]  # G
    im_array[:, 2] = 0.6 * im_array[:, 2]  # B
    return Image.frombytes('RGB', (image.width, image.height), im_array.astype(np.uint8))
    

def setup_digit_images_123rf():
    image = Image.open(os.path.join(work_dir, 'flipdigits_123rf.jpg'))
    digits = [[]] * 10
    for digit in range(10):
        x = 215 + digit * 597
        y = 1332
        w = 560
        h = 850
        im = image.crop((x, y, x + w, y + h))
        digits[digit] = colorize_image(im.resize((70, 120), Image.BICUBIC))
    # Read transition image lists.
    transition_image_lists = collections.defaultdict(list)
    for frame in range(3):
        for digit in range(10):
            x = 215 + digit * 597
            y = 1332 + 900 * (frame + 1)
            w = 560
            h = 850
            im = image.crop((x, y, x + w, y + h))
            transition_image_lists[(digit, (digit + 1) % 10)].append(
                colorize_image(im.resize((70, 120), Image.BICUBIC)))
    # Special cases for 2359 -> 0000
    for index, digit_pair in enumerate([(2, 0), (3, 0), (5, 0)]):
        x = 215 + 112 + 597 * (10 + index)
        w = 560
        h = 850
        for frame in range(3):
            y = 1332 + 900 * (frame + 1)
            im = image.crop((x, y, x + w, y + h))
            transition_image_lists[digit_pair].append(
                colorize_image(im.resize((70, 120), Image.BICUBIC)))
    return digits, transition_image_lists

# Render a display.
def make_image(time):
    """Construct an image based on message.

    Args:
      message: Integer in range 0..9999.
      digit_images: A list of 10 images, one per digit.

    Returns:
      new Image consisting of the message from the images.
    """
    im = Image.new("RGB", (320, 240))
    divisor = 1000
    for digit_number in range(4):
        digit = (time // divisor) % 10
        im.paste(digit_images[digit], (digit_number * 74 + 20 * (digit_number > 1), 40))
        divisor = divisor // 10
    return im

def make_transition_image(last_time, now_time, frame):
    #return make_image(now_time)
    im = Image.new("RGB", (320, 240))
    divisor = 1000
    for digit_number in range(4):
        old_digit = (last_time // divisor) % 10
        new_digit = (now_time // divisor) % 10
        divisor = divisor // 10
        digit_im = digit_images[new_digit]
        if (old_digit, new_digit) in transition_image_lists:
            image_list = transition_image_lists[(old_digit, new_digit)]
            if frame < len(image_list):
                digit_im = image_list[frame]
        im.paste(digit_im, (digit_number * 74 + 20 * (digit_number > 1), 40))
    return im

digit_images, transition_image_lists = setup_digit_images_123rf()
LAST_FRAME = 4

# Render the time.
last_time = 9999
frame = -1
while True:
    if frame > -1:
        # We're mid-animation.
        image = make_transition_image(last_time, now_time, frame)
        disp.image(image)
        frame += 1
        if frame == LAST_FRAME:
            frame = -1
            last_time = now_time
    else:
        now = datetime.now()
        now_time = 100 * now.hour + now.minute
        if now_time != last_time:
            # Start the transition.
            frame = 0
        time.sleep(0.1)
