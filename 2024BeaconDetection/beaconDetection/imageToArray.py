from PIL import Image
from findBrightestRowColumn import find_brightest_pixel

def image_to_pixel_array(image_path):
    # Open the image file
    image = Image.open(image_path)

    # Convert the image to grayscale
    grayscale_image = image.convert('L')

    # Get the pixel values as a 2D array
    pixel_array = list(grayscale_image.getdata())
    width, height = grayscale_image.size
    pixel_array = [pixel_array[i * width:(i + 1) * width] for i in range(height)]

    return pixel_array


# Example usage:
print("Brightest pixel coordinates:", find_brightest_pixel(pixel_array=image_to_pixel_array("C:\\Users\\doggl/Desktop\\raspibullshit\\beacondetection\\example.png")))
print("Brightest pixel coordinates:", find_brightest_pixel(pixel_array=image_to_pixel_array("C:\\Users\\doggl/Desktop\\raspibullshit\\beacondetection\\example2.png")))
print("Brightest pixel coordinates:", find_brightest_pixel(pixel_array=image_to_pixel_array("C:\\Users\\doggl/Desktop\\raspibullshit\\beacondetection\\example3.png")))

