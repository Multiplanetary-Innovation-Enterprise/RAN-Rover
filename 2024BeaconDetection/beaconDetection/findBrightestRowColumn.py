def find_brightest_pixel(pixel_array):
    max_brightness = 0
    brightest_pixel = None

    for row_index, row in enumerate(pixel_array):
        for col_index, brightness in enumerate(row):
            if brightness > max_brightness:
                max_brightness = brightness
                brightest_pixel = (col_index, row_index)

    return brightest_pixel
