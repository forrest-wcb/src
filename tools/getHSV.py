import cv2
import numpy as np

def on_trackbar(val):
    pass

def main():
    # Read the image
    image = cv2.imread('root/dev_ws/img/1721099212.1676838.jpg')

    # Convert the image to HSV format
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Create a window to display the image
    cv2.namedWindow('HSV Image')

    # Create trackbars for adjusting HSV values
    cv2.createTrackbar('Hue Min', 'HSV Image', 0, 179, on_trackbar)
    cv2.createTrackbar('Hue Max', 'HSV Image', 179, 179, on_trackbar)
    cv2.createTrackbar('Saturation Min', 'HSV Image', 0, 255, on_trackbar)
    cv2.createTrackbar('Saturation Max', 'HSV Image', 255, 255, on_trackbar)
    cv2.createTrackbar('Value Min', 'HSV Image', 0, 255, on_trackbar)
    cv2.createTrackbar('Value Max', 'HSV Image', 255, 255, on_trackbar)

    while True:
        # Get current trackbar values
        hue_min = cv2.getTrackbarPos('Hue Min', 'HSV Image')
        hue_max = cv2.getTrackbarPos('Hue Max', 'HSV Image')
        saturation_min = cv2.getTrackbarPos('Saturation Min', 'HSV Image')
        saturation_max = cv2.getTrackbarPos('Saturation Max', 'HSV Image')
        value_min = cv2.getTrackbarPos('Value Min', 'HSV Image')
        value_max = cv2.getTrackbarPos('Value Max', 'HSV Image')

        # Create lower and upper bounds for HSV values
        lower_bound = np.array([hue_min, saturation_min, value_min])
        upper_bound = np.array([hue_max, saturation_max, value_max])
        mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
        output = cv2.bitwise_and(image, image, mask=mask)
        cv2.imshow('HSV Image', output)
        # Print the new lower and upper bounds when trackbar values change
        
        if cv2.waitKey(1) & 0xFF == ord('p'):
            print('Lower Bound:', lower_bound)
            print('Upper Bound:', upper_bound)
        elif cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the resources
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()