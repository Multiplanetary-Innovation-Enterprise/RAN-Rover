import numpy as np

def find_brightest_pixel(pixel_array):
    beacon_range = 150   # Values can range from 0 (black) to 255 (white)
    brightest_pixels = []

    for row_index, row in enumerate(pixel_array):
        for col_index, brightness in enumerate(row):
            if brightness > beacon_range:
                brightest_pixels.append((row_index, col_index))        # brightest_pixels = ((col_index, row_index)) corresponds to a single brightness value



    # GOAL: determine how many groups of light pixels are on a dark background. Where a group means the difference in their lighting value is above 150,
    # but not different from one another by more than a threshold of 50 pixels. We will then print the number of groups.

    # Group together the groups of pixels

    beaconArea = []
    if brightest_pixels.__len__() > 0:
        beaconArea = brightest_pixels[0]

    for row_index, row in enumerate(brightest_pixels):
        for col_index, col in enumerate(brightest_pixels):

            #If the difference between the pixels is less than 50 (Example-- origin: 150, left: 164, right: 125, up: 151, down: 149)
            # We do this by checking the coordinates of the brightest pixels on the original pixel_array.
                # Then we want to add all beacon values to the beaconArea array.
            if np.diff(brightest_pixels.pop()) < 50;
            brightest_pixels.pop()


    return brightest_pixels
